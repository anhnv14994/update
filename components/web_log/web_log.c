#include "web_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "web_log";

// ---------------------------------------------------------------------------
// Ring buffer (PSRAM) capturing every ESP_LOG* line, in addition to the
// normal serial output. Lets you read recent logs from the browser without
// a USB/serial cable attached — handy for chasing intermittent issues like
// AirPlay stutter while the device is actually in use, away from a PC.
// ---------------------------------------------------------------------------
#define LOG_RING_SIZE (32 * 1024)

static char *s_ring = NULL;
static size_t s_write_pos = 0;
static bool s_filled = false;
static portMUX_TYPE s_ring_lock = portMUX_INITIALIZER_UNLOCKED;

static vprintf_like_t s_prev_vprintf = NULL;

static void ring_write(const char *data, size_t len)
{
    if (!s_ring || len == 0) return;
    if (len >= LOG_RING_SIZE) {
        // Only keep the tail if a single write is (absurdly) bigger than the ring.
        data += (len - LOG_RING_SIZE);
        len = LOG_RING_SIZE;
    }

    portENTER_CRITICAL(&s_ring_lock);
    size_t space_to_end = LOG_RING_SIZE - s_write_pos;
    if (len <= space_to_end) {
        memcpy(s_ring + s_write_pos, data, len);
        s_write_pos += len;
        if (s_write_pos == LOG_RING_SIZE) {
            s_write_pos = 0;
            s_filled = true;
        }
    } else {
        memcpy(s_ring + s_write_pos, data, space_to_end);
        memcpy(s_ring, data + space_to_end, len - space_to_end);
        s_write_pos = len - space_to_end;
        s_filled = true;
    }
    portEXIT_CRITICAL(&s_ring_lock);
}

// Returns the number of bytes written into `out` (chronological order,
// oldest first). `out` must be at least LOG_RING_SIZE bytes.
static size_t ring_read_all(char *out)
{
    size_t n;
    portENTER_CRITICAL(&s_ring_lock);
    if (!s_filled) {
        n = s_write_pos;
        memcpy(out, s_ring, n);
    } else {
        size_t first_chunk = LOG_RING_SIZE - s_write_pos; // oldest data starts at write_pos
        memcpy(out, s_ring + s_write_pos, first_chunk);
        memcpy(out + first_chunk, s_ring, s_write_pos);
        n = LOG_RING_SIZE;
    }
    portEXIT_CRITICAL(&s_ring_lock);
    return n;
}

static void ring_clear(void)
{
    portENTER_CRITICAL(&s_ring_lock);
    s_write_pos = 0;
    s_filled = false;
    portEXIT_CRITICAL(&s_ring_lock);
}

// vprintf hook: forward to the original output (serial console) unchanged,
// AND append a formatted copy into the ring buffer.
static int log_capture_vprintf(const char *fmt, va_list args)
{
    va_list args_copy;
    va_copy(args_copy, args);

    int ret = s_prev_vprintf ? s_prev_vprintf(fmt, args) : vprintf(fmt, args);

    char line[256];
    int n = vsnprintf(line, sizeof(line), fmt, args_copy);
    va_end(args_copy);
    if (n > 0) {
        if ((size_t)n >= sizeof(line)) n = sizeof(line) - 1;
        ring_write(line, (size_t)n);
    }

    return ret;
}

esp_err_t web_log_init(void)
{
    s_ring = heap_caps_malloc(LOG_RING_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_ring) {
        // Fall back to internal RAM with a smaller buffer if PSRAM alloc fails
        s_ring = malloc(4096);
        if (!s_ring) return ESP_ERR_NO_MEM;
    }
    s_prev_vprintf = esp_log_set_vprintf(log_capture_vprintf);
    ESP_LOGI(TAG, "Web log capture active (%d KB ring buffer)", LOG_RING_SIZE / 1024);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------
static esp_err_t api_log_get_handler(httpd_req_t *req)
{
    char *buf = malloc(LOG_RING_SIZE + 1);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    size_t n = ring_read_all(buf);
    buf[n] = '\0';

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, buf, n);
    free(buf);
    return ESP_OK;
}

static esp_err_t api_log_clear_handler(httpd_req_t *req)
{
    ring_clear();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const char LOG_PAGE_HTML[] =
"<!DOCTYPE html><html lang=\"vi\"><head><meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
"<title>Device Log</title><style>"
"body{margin:0;background:#0b0d10;color:#d8dee4;font-family:ui-monospace,Consolas,monospace;}"
".bar{position:sticky;top:0;display:flex;gap:10px;align-items:center;flex-wrap:wrap;"
"padding:10px 14px;background:#141820;border-bottom:1px solid #262b33;}"
".bar a{color:#7aa2f7;text-decoration:none;font-size:13px;}"
".bar b{font-size:14px;letter-spacing:1px;color:#7aa2f7;margin-right:auto;}"
"button,label{background:#1f2530;border:1px solid #2c3340;color:#d8dee4;"
"padding:6px 10px;border-radius:6px;font-size:12px;cursor:pointer;}"
"button:active{transform:scale(.96);}"
"#log{white-space:pre-wrap;word-break:break-all;padding:12px 14px;font-size:12px;"
"line-height:1.5;}"
".hl-err{color:#f38ba8;} .hl-warn{color:#f9e2af;} .hl-late{color:#fab387;font-weight:600;}"
"</style></head><body>"
"<div class=\"bar\">"
"<b>DEVICE LOG</b>"
"<label><input type=\"checkbox\" id=\"auto\" checked> Tự cuộn</label>"

"<label><input type=\"checkbox\" id=\"poll\"> Tự làm mới (bật khi cần theo dõi live)</label>"
"<button onclick=\"doClear()\">Xoá log</button>"
"<button onclick=\"fetchLog()\">Làm mới ngay</button>"
"<a href=\"/dsp\">DSP Sound</a>"
"<a href=\"/\">&larr; Cấu hình</a>"
"</div>"
"<pre id=\"log\">Đang tải...</pre>"
"<script>"
"const logEl=document.getElementById('log');"
"const autoEl=document.getElementById('auto');"
"const pollEl=document.getElementById('poll');"
"function esc(s){return s.replace(/&/g,'&amp;').replace(/</g,'&lt;');}"
"function colorize(text){"
"  return text.split('\\n').map(line=>{"
"    let cls='';"
"    if(line.indexOf('packet too late')>=0) cls='hl-late';"
"    else if(/^W /.test(line)||/^W\\(/.test(line)) cls='hl-warn';"
"    else if(/^E /.test(line)||/^E\\(/.test(line)) cls='hl-err';"
"    else if(line.indexOf('stack overflow')>=0||line.indexOf('panic')>=0) cls='hl-err';"
"    return cls? '<span class=\"'+cls+'\">'+esc(line)+'</span>' : esc(line);"
"  }).join('\\n');"
"}"
"async function fetchLog(){"
"  try{"
"    const res=await fetch('/api/log');"
"    const text=await res.text();"
"    logEl.innerHTML=colorize(text);"
"    if(autoEl.checked) window.scrollTo(0, document.body.scrollHeight);"
"  }catch(e){}"
"}"
"async function doClear(){"
"  await fetch('/api/log/clear',{method:'POST'});"
"  fetchLog();"
"}"
"fetchLog();"
"setInterval(()=>{ if(pollEl.checked) fetchLog(); }, 3000);"
"</script></body></html>";

static esp_err_t log_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, must-revalidate");
    httpd_resp_send(req, LOG_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t web_log_register(httpd_handle_t server)
{
    if (!server) return ESP_ERR_INVALID_ARG;

    static const httpd_uri_t routes[] = {
        { .uri = "/log",             .method = HTTP_GET,  .handler = log_page_handler },
        { .uri = "/api/log",         .method = HTTP_GET,  .handler = api_log_get_handler },
        { .uri = "/api/log/clear",   .method = HTTP_POST, .handler = api_log_clear_handler },
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        esp_err_t err = httpd_register_uri_handler(server, &routes[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register %s: %s", routes[i].uri, esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG, "Web log UI registered: /log + /api/log");
    return ESP_OK;
}
