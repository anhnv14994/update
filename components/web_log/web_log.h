#ifndef WEB_LOG_H
#define WEB_LOG_H

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Hook into ESP-IDF logging (esp_log_set_vprintf) and start capturing
 *        every log line into an in-RAM ring buffer (PSRAM), in addition to
 *        the normal UART/USB serial output (which keeps working unchanged).
 *        Call once, early in app_main() (right after nvs_flash_init()) so
 *        boot-time logs are captured too.
 */
esp_err_t web_log_init(void);

/**
 * @brief Register the /log page and /api/log endpoints on an already-running
 *        httpd server. Call after web_server_start().
 */
esp_err_t web_log_register(httpd_handle_t server);

#ifdef __cplusplus
}
#endif

#endif // WEB_LOG_H
