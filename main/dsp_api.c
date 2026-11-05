#include "dsp_api.h"
#include "dsp_engine.h"
#include "stereo_spatial.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern stereo_spatial_ctx_t *g_stereo_ctx;
void stereo_spatial_get_config(const stereo_spatial_ctx_t *ctx, stereo_spatial_config_t *cfg);

static const char *TAG = "dsp_api";

extern const char web_dsp_html_start[] asm("_binary_web_dsp_html_start");
extern const char web_dsp_html_end[]   asm("_binary_web_dsp_html_end");


// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t buf_len)
{
    if (req->content_len == 0) {
        buf[0] = '\0';
        return ESP_OK;
    }

    if (req->content_len >= buf_len) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request too large");
        return ESP_FAIL;
    }

    size_t received = 0;

    while (received < req->content_len) {

        int ret = httpd_req_recv(
            req,
            buf + received,
            req->content_len - received
        );

        if (ret <= 0) {

            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
                httpd_resp_send_408(req);

            return ESP_FAIL;
        }

        received += (size_t)ret;
    }

    buf[received] = '\0';

    return ESP_OK;
}


static cJSON *preset_to_json(dsp_preset_id_t id)
{
    dsp_preset_t p;

    if (dsp_engine_get_preset(id, &p) != ESP_OK)
        return NULL;


    cJSON *o =
        cJSON_CreateObject();

    cJSON_AddNumberToObject(
        o,
        "id",
        (int)id
    );

    cJSON_AddStringToObject(
        o,
        "name",
        p.name
    );

    cJSON_AddNumberToObject(
        o,
        "master_gain_db",
        p.master_gain_db
    );


    cJSON *bands =
        cJSON_CreateArray();


    for (int i = 0; i < p.num_bands; i++) {

        cJSON *b =
            cJSON_CreateObject();

        cJSON_AddNumberToObject(
            b,
            "freq",
            p.bands[i].freq
        );

        cJSON_AddNumberToObject(
            b,
            "gain_db",
            p.bands[i].gain_db
        );

        cJSON_AddNumberToObject(
            b,
            "q",
            p.bands[i].q
        );

        cJSON_AddItemToArray(
            bands,
            b
        );
    }


    cJSON_AddItemToObject(
        o,
        "bands",
        bands
    );


    cJSON *shelf =
        cJSON_CreateObject();

    cJSON_AddNumberToObject(
        shelf,
        "freq",
        p.shelf_freq
    );

    cJSON_AddNumberToObject(
        shelf,
        "gain_db",
        p.shelf_gain_db
    );

    cJSON_AddItemToObject(
        o,
        "shelf",
        shelf
    );


    return o;
}


// ---------------------------------------------------------------------------
// Build complete DSP state
// ---------------------------------------------------------------------------

static cJSON *build_state_json(void)
{
    cJSON *root =
        cJSON_CreateObject();


    // ---------------------------------------------------------
    // Main DSP
    // ---------------------------------------------------------

    cJSON_AddBoolToObject(
        root,
        "enabled",
        dsp_engine_get_enabled()
    );

    cJSON_AddNumberToObject(
        root,
        "active",
        (int)dsp_engine_get_active_preset()
    );

    cJSON_AddNumberToObject(
        root,
        "sample_rate",
        44100
    );


    // ---------------------------------------------------------
    // Stereo Spatial
    // ---------------------------------------------------------

    int spatial_mode = 0;

    bool enable_width_eq = false;
    bool enable_decorrelator = false;


    if (g_stereo_ctx) {

        stereo_spatial_config_t cfg;

        stereo_spatial_get_config(
            g_stereo_ctx,
            &cfg
        );

        spatial_mode =
            (int)cfg.mode;

        enable_width_eq =
            cfg.enable_width_eq;

        enable_decorrelator =
            cfg.enable_decorrelator;
    }


    cJSON_AddNumberToObject(
        root,
        "spatial_mode",
        spatial_mode
    );

    cJSON_AddBoolToObject(
        root,
        "enable_width_eq",
        enable_width_eq
    );

    cJSON_AddBoolToObject(
        root,
        "enable_decorrelator",
        enable_decorrelator
    );


    // ---------------------------------------------------------
    // Analog Character
    // ---------------------------------------------------------

    dsp_character_t character;
    if (dsp_engine_get_character(&character) == ESP_OK) {
        cJSON *ch = cJSON_CreateObject();
        cJSON_AddBoolToObject(ch, "enabled", character.enabled);
        cJSON_AddNumberToObject(ch, "mode", (int)character.mode);
        cJSON_AddNumberToObject(ch, "warmth_pct", character.warmth_pct);
        cJSON_AddNumberToObject(ch, "detail_pct", character.detail_pct);
        cJSON_AddNumberToObject(ch, "drive_pct", character.drive_pct);
        cJSON_AddNumberToObject(ch, "softness_pct", character.softness_pct);
        cJSON_AddNumberToObject(ch, "air_pct", character.air_pct);
        cJSON_AddNumberToObject(ch, "memory_pct", character.memory_pct);
        cJSON_AddNumberToObject(ch, "spectral_pct", character.spectral_pct);
        cJSON_AddBoolToObject(ch, "harmonic_interaction_enabled",
                              character.harmonic_interaction_enabled);
        cJSON_AddNumberToObject(ch, "harmonic_interaction_pct",
                                character.harmonic_interaction_pct);
        cJSON_AddBoolToObject(ch, "dynamic_saturation_enabled",
                              character.dynamic_saturation_enabled);
        cJSON_AddNumberToObject(ch, "dynamic_saturation_pct",
                                character.dynamic_saturation_pct);
        bool hf_a_enabled = false, hf_b_enabled = false;
        uint8_t hf_a_strength_pct = 0, hf_b_strength_pct = 0;
        (void)dsp_engine_get_character_hf(
            &hf_a_enabled, &hf_b_enabled, &hf_a_strength_pct, &hf_b_strength_pct);
        cJSON_AddBoolToObject(ch, "hf_a_enabled", hf_a_enabled);
        cJSON_AddBoolToObject(ch, "hf_b_enabled", hf_b_enabled);
        cJSON_AddNumberToObject(ch, "hf_a_strength_pct", hf_a_strength_pct);
        cJSON_AddNumberToObject(ch, "hf_b_strength_pct", hf_b_strength_pct);
        cJSON_AddItemToObject(root, "character", ch);
    }


    // ---------------------------------------------------------
    // Presets
    // ---------------------------------------------------------

    cJSON *presets =
        cJSON_CreateArray();


    for (int i = 0;
         i < DSP_NUM_PRESETS;
         i++) {

        cJSON *pj =
            preset_to_json(
                (dsp_preset_id_t)i
            );

        if (pj)
            cJSON_AddItemToArray(
                presets,
                pj
            );
    }


    cJSON_AddItemToObject(
        root,
        "presets",
        presets
    );


    return root;
}


static esp_err_t send_state(httpd_req_t *req)
{
    cJSON *root =
        build_state_json();

    char *text =
        cJSON_PrintUnformatted(root);

    cJSON_Delete(root);


    if (!text) {

        httpd_resp_send_500(req);

        return ESP_FAIL;
    }


    httpd_resp_set_type(
        req,
        "application/json"
    );

    httpd_resp_send(
        req,
        text,
        HTTPD_RESP_USE_STRLEN
    );

    free(text);

    return ESP_OK;
}


static esp_err_t send_error_json(
    httpd_req_t *req,
    int code,
    const char *msg
)
{
    httpd_resp_set_status(
        req,
        code == 400
            ? "400 Bad Request"
            : "500 Internal Server Error"
    );

    httpd_resp_set_type(
        req,
        "application/json"
    );


    char body[128];

    snprintf(
        body,
        sizeof(body),
        "{\"ok\":false,\"error\":\"%s\"}",
        msg
    );


    httpd_resp_send(
        req,
        body,
        HTTPD_RESP_USE_STRLEN
    );


    return ESP_FAIL;
}


static dsp_preset_id_t get_target_preset(
    cJSON *body
)
{
    if (!body)
        return dsp_engine_get_active_preset();


    cJSON *p =
        cJSON_GetObjectItem(
            body,
            "preset"
        );


    if (cJSON_IsNumber(p) &&
        p->valueint >= 0 &&
        p->valueint < DSP_NUM_PRESETS) {

        return (dsp_preset_id_t)
            p->valueint;
    }


    return dsp_engine_get_active_preset();
}


static cJSON *parse_request_json(
    httpd_req_t *req,
    char *buf,
    size_t buf_len
)
{
    if (read_body(req, buf, buf_len) != ESP_OK)
        return NULL;


    if (buf[0] == '\0')
        return NULL;


    return cJSON_Parse(buf);
}


// ---------------------------------------------------------------------------
// Handlers
// ---------------------------------------------------------------------------

static esp_err_t dsp_page_handler(
    httpd_req_t *req
)
{
    httpd_resp_set_type(req, "text/html");

    /* The page is embedded in firmware. Never allow a browser/proxy to keep
     * an older /dsp page after OTA or a firmware rebuild. */
    httpd_resp_set_hdr(req, "Cache-Control",
                       "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");

    httpd_resp_send(
        req,
        web_dsp_html_start,
        web_dsp_html_end - web_dsp_html_start
    );

    return ESP_OK;
}


static esp_err_t api_state_handler(
    httpd_req_t *req
)
{
    return send_state(req);
}


// ---------------------------------------------------------------------------
// DSP ENABLE
// ---------------------------------------------------------------------------

static esp_err_t api_enabled_handler(
    httpd_req_t *req
)
{
    char buf[128];

    cJSON *body =
        parse_request_json(
            req,
            buf,
            sizeof(buf)
        );


    if (!body)
        return send_error_json(
            req,
            400,
            "invalid or empty json"
        );


    cJSON *en =
        cJSON_GetObjectItem(
            body,
            "enabled"
        );


    if (!cJSON_IsBool(en)) {

        cJSON_Delete(body);

        return send_error_json(
            req,
            400,
            "missing 'enabled'"
        );
    }


    dsp_engine_set_enabled(
        cJSON_IsTrue(en)
    );


    cJSON_Delete(body);

    return send_state(req);
}


// ---------------------------------------------------------------------------
// Analog Character: profile A/B + realtime controls
// ---------------------------------------------------------------------------

static esp_err_t send_character_diag_state(httpd_req_t *req)
{
    dsp_character_diagnostics_t diag;
    if (dsp_engine_get_character_diagnostics(&diag) != ESP_OK)
        return httpd_resp_send_500(req);

    cJSON *root = cJSON_CreateObject();
    if (!root)
        return httpd_resp_send_500(req);

    cJSON_AddNumberToObject(root, "stage_mask", diag.stage_mask);
    cJSON_AddNumberToObject(root, "feature_mask", diag.feature_mask);
    cJSON_AddNumberToObject(root, "blocks", diag.blocks);
    cJSON_AddNumberToObject(root, "cpu_mhz", diag.cpu_mhz);

    cJSON *avg = cJSON_CreateArray();
    cJSON *max = cJSON_CreateArray();
    if (!avg || !max) {
        cJSON_Delete(avg);
        cJSON_Delete(max);
        cJSON_Delete(root);
        return httpd_resp_send_500(req);
    }

    for (int i = 0; i < 3; ++i) {
        cJSON_AddItemToArray(avg, cJSON_CreateNumber(diag.stage_avg_us[i]));
        cJSON_AddItemToArray(max, cJSON_CreateNumber(diag.stage_max_us[i]));
    }
    cJSON_AddItemToObject(root, "stage_avg_us", avg);
    cJSON_AddItemToObject(root, "stage_max_us", max);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json)
        return httpd_resp_send_500(req);

    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    free(json);
    return err;
}

static esp_err_t api_character_diag_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
        return send_character_diag_state(req);

    char buf[128];
    cJSON *body = parse_request_json(req, buf, sizeof(buf));
    if (!body)
        return send_error_json(req, 400, "invalid or empty json");

    cJSON *stage = cJSON_GetObjectItem(body, "stage_mask");
    cJSON *feature = cJSON_GetObjectItem(body, "feature_mask");
    if (!cJSON_IsNumber(stage) || !cJSON_IsNumber(feature) ||
        stage->valueint < 0 || stage->valueint > (int)DSP_CHARACTER_STAGE_MASK_ALL ||
        feature->valueint < 0 || feature->valueint > (int)DSP_CHARACTER_FEATURE_MASK_ALL) {
        cJSON_Delete(body);
        return send_error_json(req, 400, "stage_mask 0..3 and feature_mask 0..7 required");
    }

    esp_err_t err = dsp_engine_set_character_diagnostics(
        (uint8_t)stage->valueint, (uint8_t)feature->valueint);
    cJSON_Delete(body);

    if (err != ESP_OK)
        return send_error_json(req, 400, "invalid Character diagnostic masks");

    return send_character_diag_state(req);
}


/* ============================================================
 * Analog Character handler
 * ============================================================ */

static esp_err_t api_character_handler(httpd_req_t *req)
{
    char buf[512];
    cJSON *body = parse_request_json(req, buf, sizeof(buf));
    if (!body)
        return send_error_json(req, 400, "invalid or empty json");

    dsp_character_t current;
    if (dsp_engine_get_character(&current) != ESP_OK) {
        cJSON_Delete(body);
        return send_error_json(req, 500, "cannot read character config");
    }

    cJSON *mode = cJSON_GetObjectItem(body, "mode");
    if (mode && cJSON_IsNumber(mode)) {
        int m = mode->valueint;
        if (m == DSP_CHARACTER_TUBE) {
            current.enabled = true;
            current.mode = DSP_CHARACTER_TUBE;
            current.warmth_pct = 62.0f;
            current.detail_pct = 10.0f;
            current.drive_pct = 32.0f;
        } else if (m == DSP_CHARACTER_BRITISH) {
            current.enabled = true;
            current.mode = DSP_CHARACTER_BRITISH;
            current.warmth_pct = 34.0f;
            current.detail_pct = 18.0f;
            current.drive_pct = 18.0f;
        } else if (m == DSP_CHARACTER_CUSTOM) {
            current.mode = DSP_CHARACTER_CUSTOM;
        } else {
            cJSON_Delete(body);
            return send_error_json(req, 400, "mode must be 0, 1 or 2");
        }
    }

    cJSON *v;
    v = cJSON_GetObjectItem(body, "enabled");
    if (v && cJSON_IsBool(v)) current.enabled = cJSON_IsTrue(v);

    v = cJSON_GetObjectItem(body, "warmth_pct");
    if (v && cJSON_IsNumber(v)) current.warmth_pct = (float)v->valuedouble;
    v = cJSON_GetObjectItem(body, "detail_pct");
    if (v && cJSON_IsNumber(v)) current.detail_pct = (float)v->valuedouble;
    v = cJSON_GetObjectItem(body, "drive_pct");
    if (v && cJSON_IsNumber(v)) current.drive_pct = (float)v->valuedouble;

    v = cJSON_GetObjectItem(body, "harmonic_interaction_enabled");
    if (v && cJSON_IsBool(v))
        current.harmonic_interaction_enabled = cJSON_IsTrue(v);
    v = cJSON_GetObjectItem(body, "harmonic_interaction_pct");
    if (v && cJSON_IsNumber(v))
        current.harmonic_interaction_pct = (float)v->valuedouble;

    v = cJSON_GetObjectItem(body, "dynamic_saturation_enabled");
    if (v && cJSON_IsBool(v))
        current.dynamic_saturation_enabled = cJSON_IsTrue(v);
    v = cJSON_GetObjectItem(body, "dynamic_saturation_pct");
    if (v && cJSON_IsNumber(v))
        current.dynamic_saturation_pct = (float)v->valuedouble;

    cJSON *hf_a_item = cJSON_GetObjectItem(body, "hf_a_enabled");
    cJSON *hf_b_item = cJSON_GetObjectItem(body, "hf_b_enabled");
    cJSON *hf_a_strength_item = cJSON_GetObjectItem(body, "hf_a_strength_pct");
    cJSON *hf_b_strength_item = cJSON_GetObjectItem(body, "hf_b_strength_pct");
    const bool update_hf_a = cJSON_IsBool(hf_a_item);
    const bool update_hf_b = cJSON_IsBool(hf_b_item);
    const bool update_hf_a_strength = cJSON_IsNumber(hf_a_strength_item);
    const bool update_hf_b_strength = cJSON_IsNumber(hf_b_strength_item);

    if ((hf_a_strength_item && (!cJSON_IsNumber(hf_a_strength_item) ||
            hf_a_strength_item->valuedouble < 0.0 || hf_a_strength_item->valuedouble > 100.0)) ||
        (hf_b_strength_item && (!cJSON_IsNumber(hf_b_strength_item) ||
            hf_b_strength_item->valuedouble < 0.0 || hf_b_strength_item->valuedouble > 100.0))) {
        cJSON_Delete(body);
        return send_error_json(req, 400, "HF strength must be between 0 and 100 percent");
    }

    const bool hf_a_value = update_hf_a && cJSON_IsTrue(hf_a_item);
    const bool hf_b_value = update_hf_b && cJSON_IsTrue(hf_b_item);
    const uint8_t hf_a_strength_value = update_hf_a_strength
        ? (uint8_t)(hf_a_strength_item->valuedouble + 0.5) : 0U;
    const uint8_t hf_b_strength_value = update_hf_b_strength
        ? (uint8_t)(hf_b_strength_item->valuedouble + 0.5) : 0U;

    cJSON_Delete(body);

    if (dsp_engine_set_character(&current) != ESP_OK)
        return send_error_json(req, 400, "invalid character parameters");

    if (update_hf_a || update_hf_b || update_hf_a_strength || update_hf_b_strength) {
        bool old_a = false, old_b = false;
        uint8_t old_a_strength = 0, old_b_strength = 0;
        (void)dsp_engine_get_character_hf(
            &old_a, &old_b, &old_a_strength, &old_b_strength);
        if (dsp_engine_set_character_hf(
                update_hf_a ? hf_a_value : old_a,
                update_hf_b ? hf_b_value : old_b,
                update_hf_a_strength ? hf_a_strength_value : old_a_strength,
                update_hf_b_strength ? hf_b_strength_value : old_b_strength) != ESP_OK) {
            return send_error_json(req, 500, "cannot update HF refinement controls");
        }
    }

    return send_state(req);
}


// ---------------------------------------------------------------------------
// Spatial
// ---------------------------------------------------------------------------

static esp_err_t api_spatial_handler(
    httpd_req_t *req
)
{
    char buf[256];

    if (read_body(
            req,
            buf,
            sizeof(buf)
        ) != ESP_OK) {

        return ESP_FAIL;
    }


    cJSON *body =
        cJSON_Parse(buf);


    if (!body)
        return send_error_json(
            req,
            400,
            "invalid json"
        );


    cJSON *mode =
        cJSON_GetObjectItem(
            body,
            "mode"
        );


    if (!cJSON_IsNumber(mode) ||
        mode->valueint < 0 ||
        mode->valueint >
            STEREO_SPATIAL_MODE_WIDE) {

        cJSON_Delete(body);

        return send_error_json(
            req,
            400,
            "expected 'mode' (0: OFF, 1: NATURAL, 2: WIDE)"
        );
    }


    if (g_stereo_ctx) {

        stereo_spatial_config_t cfg;

        stereo_spatial_get_config(
            g_stereo_ctx,
            &cfg
        );


        cfg.mode =
            (stereo_spatial_mode_t)
            mode->valueint;


        cJSON *w_eq =
            cJSON_GetObjectItem(
                body,
                "enable_width_eq"
            );


        if (w_eq) {

            cfg.enable_width_eq =
                cJSON_IsTrue(w_eq) ||
                (
                    cJSON_IsNumber(w_eq) &&
                    w_eq->valueint == 1
                );
        }


        cJSON *decorr =
            cJSON_GetObjectItem(
                body,
                "enable_decorrelator"
            );


        if (decorr) {

            cfg.enable_decorrelator =
                cJSON_IsTrue(decorr) ||
                (
                    cJSON_IsNumber(decorr) &&
                    decorr->valueint == 1
                );
        }


        stereo_spatial_set_config(
            g_stereo_ctx,
            &cfg
        );
    }


    cJSON_Delete(body);

    return send_state(req);
}


// ---------------------------------------------------------------------------
// Select preset
// ---------------------------------------------------------------------------

static esp_err_t api_select_handler(
    httpd_req_t *req
)
{
    char buf[128];

    cJSON *body =
        parse_request_json(
            req,
            buf,
            sizeof(buf)
        );


    if (!body)
        return send_error_json(
            req,
            400,
            "invalid or empty json"
        );


    cJSON *preset =
        cJSON_GetObjectItem(
            body,
            "preset"
        );


    if (!cJSON_IsNumber(preset) ||
        preset->valueint < 0 ||
        preset->valueint >= DSP_NUM_PRESETS) {

        cJSON_Delete(body);

        return send_error_json(
            req,
            400,
            "invalid 'preset'"
        );
    }


    dsp_engine_select_preset(
        (dsp_preset_id_t)
        preset->valueint
    );


    cJSON_Delete(body);

    return send_state(req);
}


// ---------------------------------------------------------------------------
// PEQ Band
// ---------------------------------------------------------------------------

static esp_err_t api_band_handler(
    httpd_req_t *req
)
{
    char buf[256];

    cJSON *body =
        parse_request_json(
            req,
            buf,
            sizeof(buf)
        );


    if (!body)
        return send_error_json(
            req,
            400,
            "invalid or empty json"
        );


    cJSON *band =
        cJSON_GetObjectItem(
            body,
            "band"
        );

    cJSON *freq =
        cJSON_GetObjectItem(
            body,
            "freq"
        );

    cJSON *gain =
        cJSON_GetObjectItem(
            body,
            "gain_db"
        );

    cJSON *q =
        cJSON_GetObjectItem(
            body,
            "q"
        );


    if (!cJSON_IsNumber(band) ||
        !cJSON_IsNumber(freq) ||
        !cJSON_IsNumber(gain) ||
        !cJSON_IsNumber(q)) {

        cJSON_Delete(body);

        return send_error_json(
            req,
            400,
            "expected band, freq, gain_db, q"
        );
    }


    dsp_preset_id_t target =
        get_target_preset(body);


    esp_err_t err =
        dsp_engine_set_band(
            target,
            (uint8_t)band->valueint,
            (float)freq->valuedouble,
            (float)gain->valuedouble,
            (float)q->valuedouble
        );


    cJSON_Delete(body);


    if (err != ESP_OK)
        return send_error_json(
            req,
            400,
            "invalid band parameters"
        );


    return send_state(req);
}


// ---------------------------------------------------------------------------
// Shelf
// ---------------------------------------------------------------------------

static esp_err_t api_shelf_handler(
    httpd_req_t *req
)
{
    char buf[256];

    cJSON *body =
        parse_request_json(
            req,
            buf,
            sizeof(buf)
        );


    if (!body)
        return send_error_json(
            req,
            400,
            "invalid or empty json"
        );


    cJSON *freq =
        cJSON_GetObjectItem(
            body,
            "freq"
        );

    cJSON *gain =
        cJSON_GetObjectItem(
            body,
            "gain_db"
        );


    if (!cJSON_IsNumber(freq) ||
        !cJSON_IsNumber(gain)) {

        cJSON_Delete(body);

        return send_error_json(
            req,
            400,
            "expected freq, gain_db"
        );
    }


    dsp_preset_id_t target =
        get_target_preset(body);


    dsp_engine_set_shelf(
        target,
        (float)freq->valuedouble,
        (float)gain->valuedouble
    );


    cJSON_Delete(body);

    return send_state(req);
}


// ---------------------------------------------------------------------------
// Master
// ---------------------------------------------------------------------------

static esp_err_t api_master_handler(
    httpd_req_t *req
)
{
    char buf[128];

    cJSON *body =
        parse_request_json(
            req,
            buf,
            sizeof(buf)
        );


    if (!body)
        return send_error_json(
            req,
            400,
            "invalid or empty json"
        );


    cJSON *gain =
        cJSON_GetObjectItem(
            body,
            "gain_db"
        );


    if (!cJSON_IsNumber(gain)) {

        cJSON_Delete(body);

        return send_error_json(
            req,
            400,
            "expected gain_db"
        );
    }


    dsp_preset_id_t target =
        get_target_preset(body);


    dsp_engine_set_master(
        target,
        (float)gain->valuedouble
    );


    cJSON_Delete(body);

    return send_state(req);
}


// ---------------------------------------------------------------------------
// SAVE
//
// Save cả:
//   1. PEQ preset
//   2. Analog Character V5 Lite
//
// Vì vậy nút Save trên /dsp sẽ lưu toàn bộ cấu hình.
// ---------------------------------------------------------------------------

static esp_err_t api_save_handler(
    httpd_req_t *req
)
{
    char buf[128];

    cJSON *body =
        parse_request_json(
            req,
            buf,
            sizeof(buf)
        );


    dsp_preset_id_t target =
        get_target_preset(body);


    if (body)
        cJSON_Delete(body);


    // Save PEQ preset

    esp_err_t err =
        dsp_engine_save_preset(
            target
        );


    if (err != ESP_OK)
        return send_error_json(
            req,
            500,
            "nvs preset write failed"
        );


    err = dsp_engine_save_character();
    if (err != ESP_OK)
        return send_error_json(req, 500, "nvs character write failed");

    return send_state(req);
}


// ---------------------------------------------------------------------------
// RESET
// ---------------------------------------------------------------------------

static esp_err_t api_reset_handler(
    httpd_req_t *req
)
{
    char buf[128];

    cJSON *body =
        parse_request_json(
            req,
            buf,
            sizeof(buf)
        );


    dsp_preset_id_t target =
        get_target_preset(body);


    if (body)
        cJSON_Delete(body);


    esp_err_t err =
        dsp_engine_reset_preset(
            target
        );


    if (err != ESP_OK)
        return send_error_json(
            req,
            500,
            "reset failed"
        );


    return send_state(req);
}


// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

esp_err_t dsp_api_register(
    httpd_handle_t server
)
{
    if (!server)
        return ESP_ERR_INVALID_ARG;


    static const httpd_uri_t routes[] = {

        {
            .uri = "/dsp",
            .method = HTTP_GET,
            .handler = dsp_page_handler
        },

        {
            .uri = "/api/dsp/state",
            .method = HTTP_GET,
            .handler = api_state_handler
        },

        {
            .uri = "/api/dsp/enabled",
            .method = HTTP_POST,
            .handler = api_enabled_handler
        },

        {
            .uri = "/api/dsp/character",
            .method = HTTP_POST,
            .handler = api_character_handler
        },

        {
            .uri = "/api/dsp/character_diag",
            .method = HTTP_GET,
            .handler = api_character_diag_handler
        },

        {
            .uri = "/api/dsp/character_diag",
            .method = HTTP_POST,
            .handler = api_character_diag_handler
        },

        {
            .uri = "/api/dsp/spatial",
            .method = HTTP_POST,
            .handler = api_spatial_handler
        },

        {
            .uri = "/api/dsp/select",
            .method = HTTP_POST,
            .handler = api_select_handler
        },

        {
            .uri = "/api/dsp/band",
            .method = HTTP_POST,
            .handler = api_band_handler
        },

        {
            .uri = "/api/dsp/shelf",
            .method = HTTP_POST,
            .handler = api_shelf_handler
        },

        {
            .uri = "/api/dsp/master",
            .method = HTTP_POST,
            .handler = api_master_handler
        },

        {
            .uri = "/api/dsp/save",
            .method = HTTP_POST,
            .handler = api_save_handler
        },

        {
            .uri = "/api/dsp/reset",
            .method = HTTP_POST,
            .handler = api_reset_handler
        },
    };


    for (
        size_t i = 0;
        i < sizeof(routes) / sizeof(routes[0]);
        i++
    ) {

        esp_err_t err =
            httpd_register_uri_handler(
                server,
                &routes[i]
            );


        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "Failed to register %s: %s",
                routes[i].uri,
                esp_err_to_name(err)
            );

            return err;
        }
    }


    ESP_LOGI(
        TAG,
        "DSP API registered: /dsp + /api/dsp/*"
    );


    return ESP_OK;
}