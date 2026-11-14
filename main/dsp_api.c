#include "dsp_api.h"
#include "dsp_engine.h"
#include "stereo_spatial.h"
#include "i2s_output.h"
#include "esp_log.h"
#include "cJSON.h"
#include "nvs.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

extern stereo_spatial_ctx_t *g_stereo_ctx;
void stereo_spatial_get_config(const stereo_spatial_ctx_t *ctx, stereo_spatial_config_t *cfg);

static const char *TAG = "dsp_api";

#define DSP_OPTIONS_MAGIC   0x4453504FU /* "DSPO" */
#define DSP_OPTIONS_VERSION 6U
#define DSP_OPTIONS_NS      "dsp_opts"

/* Exact V1 layout kept for in-place migration of existing NVS records. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t hf_a_enabled;
    uint8_t hf_b_enabled;
    uint8_t hf_a_strength_pct;
    uint8_t hf_b_strength_pct;
    uint8_t dynamic_loudness_enabled;
    uint8_t spatial_mode;
    uint8_t enable_width_eq;
    uint8_t enable_decorrelator;
} dsp_preset_options_v1_blob_t;

/* V2 was used briefly before DSP master bypass was added to the snapshot. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t hf_a_enabled;
    uint8_t hf_b_enabled;
    uint8_t hf_a_strength_pct;
    uint8_t hf_b_strength_pct;
    uint8_t dynamic_loudness_enabled;
    uint8_t spatial_mode;
    uint8_t enable_width_eq;
    uint8_t enable_decorrelator;
    uint8_t character_stage_mask;
    uint8_t character_feature_mask;
    dsp_character_t character;
} dsp_preset_options_v2_blob_t;

/* Exact V3 layout for migration. V4 keeps the same binary layout. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t dsp_enabled;
    uint8_t hf_a_enabled;
    uint8_t hf_b_enabled;
    uint8_t hf_a_strength_pct;
    uint8_t hf_b_strength_pct;
    uint8_t dynamic_loudness_enabled;
    uint8_t spatial_mode;
    uint8_t enable_width_eq;
    uint8_t enable_decorrelator;
    uint8_t character_stage_mask;
    uint8_t character_feature_mask;
    dsp_character_t character;
} dsp_preset_options_v3_blob_t;

/* Exact V5 layout for migration from the previous adaptive-bass firmware. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t dsp_enabled;
    uint8_t hf_a_enabled;
    uint8_t hf_b_enabled;
    uint8_t hf_a_strength_pct;
    uint8_t hf_b_strength_pct;
    uint8_t dynamic_loudness_enabled;
    uint8_t spatial_mode;
    uint8_t enable_width_eq;
    uint8_t enable_decorrelator;
    uint8_t character_stage_mask;
    uint8_t character_feature_mask;
    dsp_character_t character;
    uint8_t dynamic_bass_max_db;
} dsp_preset_options_v5_blob_t;

/* V6 adds a per-preset 1 kHz SPL calibration point and its effective gain. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t dsp_enabled;
    uint8_t hf_a_enabled;
    uint8_t hf_b_enabled;
    uint8_t hf_a_strength_pct;
    uint8_t hf_b_strength_pct;
    uint8_t dynamic_loudness_enabled;
    uint8_t spatial_mode;
    uint8_t enable_width_eq;
    uint8_t enable_decorrelator;
    uint8_t character_stage_mask;
    uint8_t character_feature_mask;
    dsp_character_t character;
    uint8_t dynamic_bass_max_db;
    uint8_t dynamic_reference_spl_db; /* 0 = estimated fallback, otherwise 35..90 dB SPL */
    uint32_t dynamic_reference_gain_q; /* effective AirPlay x preset-master gain, x1e6 */
} dsp_preset_options_blob_t;

_Static_assert(sizeof(dsp_preset_options_v5_blob_t) != sizeof(dsp_preset_options_v3_blob_t),
               "V5 preset options must be distinct from V3/V4");
_Static_assert(sizeof(dsp_preset_options_blob_t) != sizeof(dsp_preset_options_v5_blob_t),
               "V6 preset options must be distinct from V5");

/* V1 used one shared Character record. Retain it as a one-time migration base. */
static dsp_character_t s_legacy_character;
static bool s_legacy_character_valid;

static esp_err_t save_preset_options(dsp_preset_id_t id);

static void begin_audio_config_transition(void)
{
    i2s_output_transition_begin();
}

static void end_audio_config_transition(void)
{
    i2s_output_transition_end();
}

static void apply_legacy_character(void)
{
    dsp_character_t factory = { .mode = DSP_CHARACTER_CUSTOM };
    dsp_character_t migrated = s_legacy_character_valid
        ? s_legacy_character : factory;
    /* air_pct was a legacy V4 field; only V6 reuses it for HF H2. */
    migrated.air_pct = 0.0f;

    if (dsp_engine_set_character(&migrated) != ESP_OK)
        ESP_LOGW(TAG, "Could not apply legacy/factory Character fallback");
}

static void apply_default_preset_options(void)
{
    apply_legacy_character();
    (void)dsp_engine_set_character_hf(false, false, 0U, 0U);
    (void)dsp_engine_set_dynamic_loudness(false);
    (void)dsp_engine_set_dynamic_bass_max_db(4U);
    (void)dsp_engine_set_dynamic_loudness_calibration(0U, 0.5f);
    (void)dsp_engine_set_character_diagnostics(
        DSP_CHARACTER_STAGE_MASK_ALL, DSP_CHARACTER_FEATURE_MASK_ALL);

    if (g_stereo_ctx) {
        const stereo_spatial_config_t cfg = {
            .mode = STEREO_SPATIAL_MODE_OFF,
            .enable_width_eq = false,
            .enable_decorrelator = false
        };
        stereo_spatial_set_config(g_stereo_ctx, &cfg);
    }
}

static bool preset_options_v1_valid(const dsp_preset_options_v1_blob_t *blob)
{
    return blob &&
        blob->magic == DSP_OPTIONS_MAGIC &&
        blob->version == 1U &&
        blob->hf_a_enabled <= 1U &&
        blob->hf_b_enabled <= 1U &&
        blob->dynamic_loudness_enabled <= 1U &&
        blob->spatial_mode <= STEREO_SPATIAL_MODE_WIDE &&
        blob->enable_width_eq <= 1U &&
        blob->enable_decorrelator <= 1U &&
        blob->hf_a_strength_pct <= 100U &&
        blob->hf_b_strength_pct <= 100U;
}

static esp_err_t apply_legacy_preset_options(
    dsp_preset_id_t id,
    const dsp_preset_options_v1_blob_t *old)
{
    if (!preset_options_v1_valid(old)) {
        ESP_LOGW(TAG, "Preset %d legacy options invalid; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    apply_legacy_character();

    esp_err_t err = dsp_engine_set_character_hf(
        old->hf_a_enabled != 0U, old->hf_b_enabled != 0U,
        old->hf_a_strength_pct, old->hf_b_strength_pct);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_loudness(
        old->dynamic_loudness_enabled != 0U);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_bass_max_db(4U);
    if (err != ESP_OK)
        return err;
    err = dsp_engine_set_dynamic_loudness_calibration(0U, 0.5f);
    if (err != ESP_OK)
        return err;

    (void)dsp_engine_set_character_diagnostics(
        DSP_CHARACTER_STAGE_MASK_ALL, DSP_CHARACTER_FEATURE_MASK_ALL);

    if (g_stereo_ctx) {
        const stereo_spatial_config_t cfg = {
            .mode = (stereo_spatial_mode_t)old->spatial_mode,
            .enable_width_eq = old->enable_width_eq != 0U,
            .enable_decorrelator = old->enable_decorrelator != 0U
        };
        stereo_spatial_set_config(g_stereo_ctx, &cfg);
    }

    ESP_LOGI(TAG, "Preset %d V1 options migrated to complete per-preset settings",
             (int)id);
    err = save_preset_options(id);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Preset %d V1 migration save failed: %s",
                 (int)id, esp_err_to_name(err));
    return ESP_OK;
}

static bool preset_options_v2_valid(const dsp_preset_options_v2_blob_t *blob)
{
    return blob &&
        blob->magic == DSP_OPTIONS_MAGIC &&
        blob->version == 2U &&
        blob->hf_a_enabled <= 1U &&
        blob->hf_b_enabled <= 1U &&
        blob->hf_a_strength_pct <= 100U &&
        blob->hf_b_strength_pct <= 100U &&
        blob->dynamic_loudness_enabled <= 1U &&
        blob->spatial_mode <= STEREO_SPATIAL_MODE_WIDE &&
        blob->enable_width_eq <= 1U &&
        blob->enable_decorrelator <= 1U &&
        (blob->character_stage_mask & (uint8_t)~DSP_CHARACTER_STAGE_MASK_ALL) == 0U &&
        (blob->character_feature_mask & (uint8_t)~DSP_CHARACTER_FEATURE_MASK_ALL) == 0U;
}

static esp_err_t apply_v2_preset_options(
    dsp_preset_id_t id,
    const dsp_preset_options_v2_blob_t *old)
{
    if (!preset_options_v2_valid(old)) {
        ESP_LOGW(TAG, "Preset %d V2 options invalid; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    dsp_character_t migrated_character = old->character;
    migrated_character.air_pct = 0.0f; /* Reserved legacy Air control; V6 HF Air starts OFF. */
    esp_err_t err = dsp_engine_set_character(&migrated_character);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Preset %d V2 Character invalid; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    err = dsp_engine_set_character_hf(
        old->hf_a_enabled != 0U, old->hf_b_enabled != 0U,
        old->hf_a_strength_pct, old->hf_b_strength_pct);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_loudness(old->dynamic_loudness_enabled != 0U);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_bass_max_db(4U);
    if (err != ESP_OK)
        return err;
    err = dsp_engine_set_dynamic_loudness_calibration(0U, 0.5f);
    if (err != ESP_OK)
        return err;

    /* Firmware upgrade clears any diagnostic bypass left by old A/B tests. */
    err = dsp_engine_set_character_diagnostics(
        DSP_CHARACTER_STAGE_MASK_ALL, DSP_CHARACTER_FEATURE_MASK_ALL);
    if (err != ESP_OK)
        return err;

    if (g_stereo_ctx) {
        const stereo_spatial_config_t cfg = {
            .mode = (stereo_spatial_mode_t)old->spatial_mode,
            .enable_width_eq = old->enable_width_eq != 0U,
            .enable_decorrelator = old->enable_decorrelator != 0U
        };
        stereo_spatial_set_config(g_stereo_ctx, &cfg);
    }

    ESP_LOGI(TAG, "Preset %d V2 options migrated to V6 complete snapshot", (int)id);
    err = save_preset_options(id);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Preset %d V2 migration save failed: %s",
                 (int)id, esp_err_to_name(err));
    return ESP_OK;
}

static bool preset_options_v3_valid(const dsp_preset_options_v3_blob_t *blob)
{
    return blob &&
        blob->magic == DSP_OPTIONS_MAGIC &&
        (blob->version == 3U || blob->version == 4U) &&
        blob->dsp_enabled <= 1U &&
        blob->hf_a_enabled <= 1U &&
        blob->hf_b_enabled <= 1U &&
        blob->hf_a_strength_pct <= 100U &&
        blob->hf_b_strength_pct <= 100U &&
        blob->dynamic_loudness_enabled <= 1U &&
        blob->spatial_mode <= STEREO_SPATIAL_MODE_WIDE &&
        blob->enable_width_eq <= 1U &&
        blob->enable_decorrelator <= 1U &&
        (blob->version == 3U ||
         (((blob->character_stage_mask & (uint8_t)~DSP_CHARACTER_STAGE_MASK_ALL) == 0U) &&
          ((blob->character_feature_mask & (uint8_t)~DSP_CHARACTER_FEATURE_MASK_ALL) == 0U)));
}

static esp_err_t apply_v3_preset_options(
    dsp_preset_id_t id,
    const dsp_preset_options_v3_blob_t *old)
{
    if (!preset_options_v3_valid(old)) {
        ESP_LOGW(TAG, "Preset %d V3 options invalid; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    dsp_character_t migrated_character = old->character;
    migrated_character.air_pct = 0.0f; /* Prevent old V4 Air setting becoming HF H2. */
    esp_err_t err = dsp_engine_set_character(&migrated_character);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Preset %d V3 Character invalid; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    err = dsp_engine_set_character_hf(
        old->hf_a_enabled != 0U, old->hf_b_enabled != 0U,
        old->hf_a_strength_pct, old->hf_b_strength_pct);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_loudness(
        old->dynamic_loudness_enabled != 0U);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_bass_max_db(4U);
    if (err != ESP_OK)
        return err;
    err = dsp_engine_set_dynamic_loudness_calibration(0U, 0.5f);
    if (err != ESP_OK)
        return err;

    /* V3 had no active A/B mask; V4 already persisted both masks. */
    const uint8_t stage_mask = (old->version == 3U)
        ? DSP_CHARACTER_STAGE_MASK_ALL : old->character_stage_mask;
    const uint8_t feature_mask = (old->version == 3U)
        ? DSP_CHARACTER_FEATURE_MASK_ALL : old->character_feature_mask;
    err = dsp_engine_set_character_diagnostics(stage_mask, feature_mask);
    if (err != ESP_OK)
        return err;

    if (g_stereo_ctx) {
        const stereo_spatial_config_t cfg = {
            .mode = (stereo_spatial_mode_t)old->spatial_mode,
            .enable_width_eq = old->enable_width_eq != 0U,
            .enable_decorrelator = old->enable_decorrelator != 0U
        };
        stereo_spatial_set_config(g_stereo_ctx, &cfg);
    }

    if (dsp_engine_get_enabled() != (old->dsp_enabled != 0U)) {
        err = dsp_engine_set_enabled(old->dsp_enabled != 0U);
        if (err != ESP_OK)
            return err;
    }

    ESP_LOGW(TAG, "Preset %d V%u -> V6 options migration",
             (int)id, old->version);
    err = save_preset_options(id);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Preset %d V5 migration save failed: %s",
                 (int)id, esp_err_to_name(err));
    return ESP_OK;
}


static bool preset_options_v5_valid(const dsp_preset_options_v5_blob_t *blob)
{
    return blob &&
        blob->magic == DSP_OPTIONS_MAGIC &&
        blob->version == 5U &&
        blob->dsp_enabled <= 1U &&
        blob->hf_a_enabled <= 1U &&
        blob->hf_b_enabled <= 1U &&
        blob->hf_a_strength_pct <= 100U &&
        blob->hf_b_strength_pct <= 100U &&
        blob->dynamic_loudness_enabled <= 1U &&
        blob->dynamic_bass_max_db <= 5U &&
        blob->spatial_mode <= STEREO_SPATIAL_MODE_WIDE &&
        blob->enable_width_eq <= 1U &&
        blob->enable_decorrelator <= 1U &&
        (blob->character_stage_mask & (uint8_t)~DSP_CHARACTER_STAGE_MASK_ALL) == 0U &&
        (blob->character_feature_mask & (uint8_t)~DSP_CHARACTER_FEATURE_MASK_ALL) == 0U;
}

static esp_err_t apply_v5_preset_options(
    dsp_preset_id_t id,
    const dsp_preset_options_v5_blob_t *old)
{
    if (!preset_options_v5_valid(old)) {
        ESP_LOGW(TAG, "Preset %d V5 options invalid; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    dsp_character_t migrated_character = old->character;
    migrated_character.air_pct = 0.0f; /* V5 did not have the V6 HF Air control. */
    esp_err_t err = dsp_engine_set_character(&migrated_character);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Preset %d V5 Character invalid; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    err = dsp_engine_set_character_hf(
        old->hf_a_enabled != 0U, old->hf_b_enabled != 0U,
        old->hf_a_strength_pct, old->hf_b_strength_pct);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_loudness(old->dynamic_loudness_enabled != 0U);
    if (err != ESP_OK)
        return err;
    err = dsp_engine_set_dynamic_bass_max_db(old->dynamic_bass_max_db);
    if (err != ESP_OK)
        return err;

    /* V5 had no SPL calibration; retain its adaptive shape with an estimated 60-phon anchor. */
    err = dsp_engine_set_dynamic_loudness_calibration(0U, 0.5f);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_character_diagnostics(
        old->character_stage_mask, old->character_feature_mask);
    if (err != ESP_OK)
        return err;

    if (g_stereo_ctx) {
        const stereo_spatial_config_t cfg = {
            .mode = (stereo_spatial_mode_t)old->spatial_mode,
            .enable_width_eq = old->enable_width_eq != 0U,
            .enable_decorrelator = old->enable_decorrelator != 0U
        };
        stereo_spatial_set_config(g_stereo_ctx, &cfg);
    }

    if (dsp_engine_get_enabled() != (old->dsp_enabled != 0U)) {
        err = dsp_engine_set_enabled(old->dsp_enabled != 0U);
        if (err != ESP_OK)
            return err;
    }

    ESP_LOGW(TAG, "Preset %d V5 -> V6 migration: SPL calibration not yet set",
             (int)id);
    err = save_preset_options(id);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Preset %d V6 migration save failed: %s",
                 (int)id, esp_err_to_name(err));
    return ESP_OK;
}

static esp_err_t apply_preset_options(dsp_preset_id_t id)
{
    if (id < 0 || id >= DSP_NUM_PRESETS)
        return ESP_ERR_INVALID_ARG;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(DSP_OPTIONS_NS, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            apply_default_preset_options();
            return ESP_OK;
        }
        return err;
    }

    char key[8];
    snprintf(key, sizeof(key), "o%d", (int)id);

    size_t len = 0U;
    err = nvs_get_blob(nvs, key, NULL, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        apply_default_preset_options();
        return ESP_OK;
    }
    if (err != ESP_OK) {
        nvs_close(nvs);
        return err;
    }

    if (len == sizeof(dsp_preset_options_v1_blob_t)) {
        dsp_preset_options_v1_blob_t old;
        size_t read_len = sizeof(old);
        err = nvs_get_blob(nvs, key, &old, &read_len);
        nvs_close(nvs);
        if (err != ESP_OK || read_len != sizeof(old)) {
            apply_default_preset_options();
            return ESP_OK;
        }
        return apply_legacy_preset_options(id, &old);
    }

    if (len == sizeof(dsp_preset_options_v2_blob_t)) {
        dsp_preset_options_v2_blob_t old;
        size_t read_len = sizeof(old);
        err = nvs_get_blob(nvs, key, &old, &read_len);
        nvs_close(nvs);
        if (err != ESP_OK || read_len != sizeof(old)) {
            apply_default_preset_options();
            return ESP_OK;
        }
        return apply_v2_preset_options(id, &old);
    }

    if (len == sizeof(dsp_preset_options_v3_blob_t)) {
        /* V3 and V4 share the same frozen binary layout. */
        dsp_preset_options_v3_blob_t old;
        size_t read_len = sizeof(old);
        err = nvs_get_blob(nvs, key, &old, &read_len);
        nvs_close(nvs);
        if (err != ESP_OK || read_len != sizeof(old)) {
            apply_default_preset_options();
            return ESP_OK;
        }
        if (old.magic == DSP_OPTIONS_MAGIC &&
            (old.version == 3U || old.version == 4U))
            return apply_v3_preset_options(id, &old);

        ESP_LOGW(TAG, "Preset %d legacy options version unsupported; using defaults",
                 (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    if (len == sizeof(dsp_preset_options_v5_blob_t)) {
        dsp_preset_options_v5_blob_t old;
        size_t read_len = sizeof(old);
        err = nvs_get_blob(nvs, key, &old, &read_len);
        nvs_close(nvs);
        if (err != ESP_OK || read_len != sizeof(old)) {
            apply_default_preset_options();
            return ESP_OK;
        }
        return apply_v5_preset_options(id, &old);
    }

    if (len != sizeof(dsp_preset_options_blob_t)) {
        nvs_close(nvs);
        ESP_LOGW(TAG, "Preset %d options size unsupported; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    dsp_preset_options_blob_t blob;
    size_t read_len = sizeof(blob);
    err = nvs_get_blob(nvs, key, &blob, &read_len);
    nvs_close(nvs);
    if (err != ESP_OK || read_len != sizeof(blob)) {
        apply_default_preset_options();
        return ESP_OK;
    }

    if (blob.magic != DSP_OPTIONS_MAGIC || blob.version != DSP_OPTIONS_VERSION) {
        ESP_LOGW(TAG, "Preset %d options version unsupported; preserved, defaults applied",
                 (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    if (blob.dsp_enabled > 1U ||
        blob.hf_a_enabled > 1U || blob.hf_b_enabled > 1U ||
        blob.hf_a_strength_pct > 100U || blob.hf_b_strength_pct > 100U ||
        blob.dynamic_loudness_enabled > 1U ||
        blob.dynamic_bass_max_db > 5U ||
        (blob.dynamic_reference_spl_db != 0U &&
         (blob.dynamic_reference_spl_db < 35U || blob.dynamic_reference_spl_db > 80U)) ||
        blob.dynamic_reference_gain_q < 1000U ||
        blob.dynamic_reference_gain_q > 2000000U ||
        blob.spatial_mode > STEREO_SPATIAL_MODE_WIDE ||
        blob.enable_width_eq > 1U || blob.enable_decorrelator > 1U ||
        (blob.character_stage_mask & (uint8_t)~DSP_CHARACTER_STAGE_MASK_ALL) != 0U ||
        (blob.character_feature_mask & (uint8_t)~DSP_CHARACTER_FEATURE_MASK_ALL) != 0U) {
        ESP_LOGW(TAG, "Preset %d options invalid; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    if (blob.character_stage_mask != DSP_CHARACTER_STAGE_MASK_ALL ||
        blob.character_feature_mask != DSP_CHARACTER_FEATURE_MASK_ALL) {
        ESP_LOGW(TAG,
                 "Preset %d Character A/B diagnostic bypass active: stage=%u feature=%u",
                 (int)id, blob.character_stage_mask, blob.character_feature_mask);
    }

    err = dsp_engine_set_character(&blob.character);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Preset %d Character data invalid; using defaults", (int)id);
        apply_default_preset_options();
        return ESP_OK;
    }

    err = dsp_engine_set_character_hf(
        blob.hf_a_enabled != 0U, blob.hf_b_enabled != 0U,
        blob.hf_a_strength_pct, blob.hf_b_strength_pct);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_loudness(
        blob.dynamic_loudness_enabled != 0U);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_bass_max_db(blob.dynamic_bass_max_db);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_dynamic_loudness_calibration(
        blob.dynamic_reference_spl_db,
        (float)blob.dynamic_reference_gain_q * 0.000001f);
    if (err != ESP_OK)
        return err;

    err = dsp_engine_set_character_diagnostics(
        blob.character_stage_mask, blob.character_feature_mask);
    if (err != ESP_OK)
        return err;

    if (g_stereo_ctx) {
        const stereo_spatial_config_t cfg = {
            .mode = (stereo_spatial_mode_t)blob.spatial_mode,
            .enable_width_eq = blob.enable_width_eq != 0U,
            .enable_decorrelator = blob.enable_decorrelator != 0U
        };
        stereo_spatial_set_config(g_stereo_ctx, &cfg);
    }

    if (dsp_engine_get_enabled() != (blob.dsp_enabled != 0U)) {
        err = dsp_engine_set_enabled(blob.dsp_enabled != 0U);
        if (err != ESP_OK)
            return err;
    }

    ESP_LOGI(TAG,
             "Preset %d loaded: DSP=%u Character=%u HF=%u/%u Loudness=%u Spatial=%u A/B=%u/%u",
             (int)id, blob.dsp_enabled, blob.character.enabled ? 1U : 0U,
             blob.hf_a_enabled, blob.hf_b_enabled,
             blob.dynamic_loudness_enabled, blob.spatial_mode,
             blob.character_stage_mask, blob.character_feature_mask);
    return ESP_OK;
}

static esp_err_t save_preset_options(dsp_preset_id_t id)
{
    if (id < 0 || id >= DSP_NUM_PRESETS)
        return ESP_ERR_INVALID_ARG;

    dsp_preset_options_blob_t blob;
    memset(&blob, 0, sizeof(blob));
    blob.magic = DSP_OPTIONS_MAGIC;
    blob.version = DSP_OPTIONS_VERSION;
    blob.dsp_enabled = dsp_engine_get_enabled() ? 1U : 0U;

    esp_err_t err = dsp_engine_get_character(&blob.character);
    if (err != ESP_OK)
        return err;

    bool hf_a = false, hf_b = false;
    uint8_t hf_a_strength = 0U, hf_b_strength = 0U;
    err = dsp_engine_get_character_hf(
        &hf_a, &hf_b, &hf_a_strength, &hf_b_strength);
    if (err != ESP_OK)
        return err;

    blob.hf_a_enabled = hf_a ? 1U : 0U;
    blob.hf_b_enabled = hf_b ? 1U : 0U;
    blob.hf_a_strength_pct = hf_a_strength;
    blob.hf_b_strength_pct = hf_b_strength;
    blob.dynamic_loudness_enabled =
        dsp_engine_get_dynamic_loudness() ? 1U : 0U;
    blob.dynamic_bass_max_db = dsp_engine_get_dynamic_bass_max_db();
    float dynamic_reference_gain = 0.5f;
    uint8_t dynamic_reference_spl = 0U;
    dsp_engine_get_dynamic_loudness_calibration(
        &dynamic_reference_spl, &dynamic_reference_gain);
    blob.dynamic_reference_spl_db = dynamic_reference_spl;
    blob.dynamic_reference_gain_q =
        (uint32_t)(dynamic_reference_gain * 1000000.0f + 0.5f);

    err = dsp_engine_get_character_diagnostic_masks(
        &blob.character_stage_mask, &blob.character_feature_mask);
    if (err != ESP_OK)
        return err;

    if (g_stereo_ctx) {
        stereo_spatial_config_t cfg;
        stereo_spatial_get_config(g_stereo_ctx, &cfg);
        blob.spatial_mode = (uint8_t)cfg.mode;
        blob.enable_width_eq = cfg.enable_width_eq ? 1U : 0U;
        blob.enable_decorrelator = cfg.enable_decorrelator ? 1U : 0U;
    }

    nvs_handle_t nvs;
    err = nvs_open(DSP_OPTIONS_NS, NVS_READWRITE, &nvs);
    if (err != ESP_OK)
        return err;

    char key[8];
    snprintf(key, sizeof(key), "o%d", (int)id);
    err = nvs_set_blob(nvs, key, &blob, sizeof(blob));
    if (err == ESP_OK)
        err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Preset %d saved: DSP=%u Character=%u HF=%u/%u Loudness=%u Spatial=%u A/B=%u/%u",
                 (int)id, blob.dsp_enabled, blob.character.enabled ? 1U : 0U,
                 blob.hf_a_enabled, blob.hf_b_enabled,
                 blob.dynamic_loudness_enabled, blob.spatial_mode,
                 blob.character_stage_mask, blob.character_feature_mask);
    }
    return err;
}

static esp_err_t reset_preset_options(dsp_preset_id_t id)
{
    if (id < 0 || id >= DSP_NUM_PRESETS)
        return ESP_ERR_INVALID_ARG;

    dsp_preset_options_blob_t blob;
    memset(&blob, 0, sizeof(blob));
    blob.magic = DSP_OPTIONS_MAGIC;
    blob.version = DSP_OPTIONS_VERSION;
    blob.dsp_enabled = 1U;
    blob.character.mode = DSP_CHARACTER_CUSTOM;
    blob.dynamic_bass_max_db = 4U;
    blob.dynamic_reference_spl_db = 0U;
    blob.dynamic_reference_gain_q = 500000U;
    blob.character_stage_mask = DSP_CHARACTER_STAGE_MASK_ALL;
    blob.character_feature_mask = DSP_CHARACTER_FEATURE_MASK_ALL;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(DSP_OPTIONS_NS, NVS_READWRITE, &nvs);
    if (err != ESP_OK)
        return err;

    char key[8];
    snprintf(key, sizeof(key), "o%d", (int)id);
    err = nvs_set_blob(nvs, key, &blob, sizeof(blob));
    if (err == ESP_OK)
        err = nvs_commit(nvs);
    nvs_close(nvs);

    if (err == ESP_OK && id == dsp_engine_get_active_preset())
        err = apply_preset_options(id);
    return err;
}

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

    cJSON_AddNumberToObject(root, "airplay_volume",
                            dsp_engine_get_output_volume());
    cJSON_AddBoolToObject(root, "dynamic_loudness_enabled",
                          dsp_engine_get_dynamic_loudness());
    cJSON_AddNumberToObject(root, "dynamic_bass_max_db",
                            dsp_engine_get_dynamic_bass_max_db());
    uint8_t dynamic_reference_spl = 0U;
    dsp_engine_get_dynamic_loudness_calibration(
        &dynamic_reference_spl, NULL);
    cJSON_AddBoolToObject(root, "dynamic_calibrated",
                          dynamic_reference_spl >= 35U && dynamic_reference_spl <= 80U);
    cJSON_AddNumberToObject(root, "dynamic_reference_spl_db",
                            dynamic_reference_spl);


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
        cJSON_AddNumberToObject(ch, "hf_air_pct", character.air_pct);
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


    begin_audio_config_transition();
    esp_err_t set_err = dsp_engine_set_enabled(cJSON_IsTrue(en));
    end_audio_config_transition();

    cJSON_Delete(body);
    if (set_err != ESP_OK)
        return send_error_json(req, 500, "cannot update DSP enabled state");

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

    cJSON *reset = cJSON_GetObjectItem(body, "reset");
    if (cJSON_IsTrue(reset)) {
        cJSON_Delete(body);
        begin_audio_config_transition();
        esp_err_t reset_err = dsp_engine_set_character_diagnostics(
            DSP_CHARACTER_STAGE_MASK_ALL, DSP_CHARACTER_FEATURE_MASK_ALL);
        end_audio_config_transition();
        if (reset_err == ESP_OK)
            reset_err = save_preset_options(dsp_engine_get_active_preset());
        if (reset_err != ESP_OK)
            return send_error_json(req, 500,
                                   "cannot reset and save Character diagnostic masks");
        ESP_LOGI(TAG, "Character diagnostic masks reset to ALL and saved to active preset");
        return send_character_diag_state(req);
    }

    cJSON *stage = cJSON_GetObjectItem(body, "stage_mask");
    cJSON *feature = cJSON_GetObjectItem(body, "feature_mask");
    if (!cJSON_IsNumber(stage) || !cJSON_IsNumber(feature) ||
        stage->valueint < 0 || stage->valueint > (int)DSP_CHARACTER_STAGE_MASK_ALL ||
        feature->valueint < 0 || feature->valueint > (int)DSP_CHARACTER_FEATURE_MASK_ALL) {
        cJSON_Delete(body);
        return send_error_json(req, 400,
                               "stage_mask 0..3 and feature_mask 0..7 required, or reset=true");
    }

    begin_audio_config_transition();
    esp_err_t err = dsp_engine_set_character_diagnostics(
        (uint8_t)stage->valueint, (uint8_t)feature->valueint);
    end_audio_config_transition();
    cJSON_Delete(body);

    if (err != ESP_OK)
        return send_error_json(req, 400, "invalid Character diagnostic masks");

    return send_character_diag_state(req);
}


/* ============================================================
 * Dynamic Loudness handler (saved with the active preset on Save)
 * ============================================================ */

static esp_err_t api_loudness_handler(httpd_req_t *req)
{
    char buf[192];
    cJSON *body = parse_request_json(req, buf, sizeof(buf));
    if (!body)
        return send_error_json(req, 400, "invalid or empty json");

    cJSON *enabled = cJSON_GetObjectItem(body, "enabled");
    cJSON *bass_max = cJSON_GetObjectItem(body, "bass_max_db");
    cJSON *calibrate = cJSON_GetObjectItem(body, "calibrate");
    cJSON *reference_spl = cJSON_GetObjectItem(body, "reference_spl_db");

    if ((enabled && !cJSON_IsBool(enabled)) ||
        (bass_max && (!cJSON_IsNumber(bass_max) ||
         bass_max->valuedouble < 0.0 || bass_max->valuedouble > 5.0 ||
         (double)bass_max->valueint != bass_max->valuedouble)) ||
        (calibrate && !cJSON_IsBool(calibrate)) ||
        (reference_spl && (!cJSON_IsNumber(reference_spl) ||
         reference_spl->valuedouble < 35.0 || reference_spl->valuedouble > 80.0 ||
         (double)reference_spl->valueint != reference_spl->valuedouble))) {
        cJSON_Delete(body);
        return send_error_json(req, 400, "invalid loudness setting or SPL calibration value");
    }

    const bool has_enabled = enabled != NULL;
    const bool enabled_value = has_enabled && cJSON_IsTrue(enabled);
    const bool has_bass_max = bass_max != NULL;
    const uint8_t bass_max_value = has_bass_max ? (uint8_t)bass_max->valueint : 0U;
    const bool calibrate_now = cJSON_IsTrue(calibrate);
    const bool has_reference_spl = reference_spl != NULL;
    const uint8_t reference_spl_value =
        has_reference_spl ? (uint8_t)reference_spl->valueint : 0U;
    cJSON_Delete(body);

    begin_audio_config_transition();
    esp_err_t err = ESP_OK;
    if (has_enabled)
        err = dsp_engine_set_dynamic_loudness(enabled_value);
    if (err == ESP_OK && has_bass_max)
        err = dsp_engine_set_dynamic_bass_max_db(bass_max_value);

    if (err == ESP_OK && calibrate_now) {
        if (!has_reference_spl) {
            err = ESP_ERR_INVALID_ARG;
        } else {
            dsp_preset_t preset;
            err = dsp_engine_get_preset(
                dsp_engine_get_active_preset(), &preset);
            if (err == ESP_OK) {
                const float master_gain = powf(10.0f, preset.master_gain_db / 20.0f);
                const float calibration_gain =
                    dsp_engine_get_output_volume() * master_gain;
                err = dsp_engine_set_dynamic_loudness_calibration(
                    reference_spl_value, calibration_gain);
            }
        }
    }
    end_audio_config_transition();

    if (err != ESP_OK)
        return send_error_json(req, 400,
            "cannot update loudness settings; calibration requires nonzero AirPlay volume");

    return send_state(req);
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
    const dsp_character_t previous = current;

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
    v = cJSON_GetObjectItem(body, "hf_air_pct");
    if (v && cJSON_IsNumber(v)) current.air_pct = (float)v->valuedouble;

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

    bool old_a = false, old_b = false;
    uint8_t old_a_strength = 0, old_b_strength = 0;
    (void)dsp_engine_get_character_hf(
        &old_a, &old_b, &old_a_strength, &old_b_strength);

    const bool hf_switch_changed =
        (update_hf_a && hf_a_value != old_a) ||
        (update_hf_b && hf_b_value != old_b);
    const bool needs_transition =
        current.enabled != previous.enabled ||
        current.mode != previous.mode ||
        hf_switch_changed;

    if (needs_transition)
        begin_audio_config_transition();

    esp_err_t err = dsp_engine_set_character(&current);
    if (err == ESP_OK &&
        (update_hf_a || update_hf_b || update_hf_a_strength || update_hf_b_strength)) {
        err = dsp_engine_set_character_hf(
            update_hf_a ? hf_a_value : old_a,
            update_hf_b ? hf_b_value : old_b,
            update_hf_a_strength ? hf_a_strength_value : old_a_strength,
            update_hf_b_strength ? hf_b_strength_value : old_b_strength);
    }

    if (needs_transition)
        end_audio_config_transition();

    if (err != ESP_OK)
        return send_error_json(req, 400, "invalid Character/HF parameters");

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


        begin_audio_config_transition();
        stereo_spatial_set_config(
            g_stereo_ctx,
            &cfg
        );
        end_audio_config_transition();
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


    const dsp_preset_id_t selected = (dsp_preset_id_t)preset->valueint;
    begin_audio_config_transition();
    esp_err_t err = dsp_engine_select_preset(selected);
    if (err == ESP_OK)
        err = apply_preset_options(selected);
    end_audio_config_transition();

    cJSON_Delete(body);
    if (err != ESP_OK)
        return send_error_json(req, 500, "cannot load preset options");

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


    esp_err_t err = dsp_engine_set_shelf(
        target,
        (float)freq->valuedouble,
        (float)gain->valuedouble
    );

    cJSON_Delete(body);
    if (err != ESP_OK)
        return send_error_json(req, 400, "invalid shelf parameters");

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


    esp_err_t err = dsp_engine_set_master(
        target,
        (float)gain->valuedouble
    );

    cJSON_Delete(body);
    if (err != ESP_OK)
        return send_error_json(req, 400, "invalid master gain");

    return send_state(req);
}


// ---------------------------------------------------------------------------
// SAVE
//
// Save DSP bypass, PEQ, master gain, Analog Character, HF A/B,
// Dynamic Loudness, Stereo Spatial and Character A/B masks as one snapshot.
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


    err = save_preset_options(target);
    if (err != ESP_OK)
        return send_error_json(req, 500, "nvs preset options write failed");

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


    const bool active_target = (target == dsp_engine_get_active_preset());
    if (active_target)
        begin_audio_config_transition();

    esp_err_t err =
        dsp_engine_reset_preset(
            target
        );

    if (err == ESP_OK)
        err = reset_preset_options(target);

    if (active_target)
        end_audio_config_transition();

    if (err != ESP_OK)
        return send_error_json(req, 500, "preset reset failed");

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

    /* Capture the old shared Character once for V1/missing-preset migration. */
    s_legacy_character_valid =
        (dsp_engine_get_character(&s_legacy_character) == ESP_OK);

    /* NVS + Stereo Spatial are initialized before the DSP routes register. */
    esp_err_t options_err = apply_preset_options(dsp_engine_get_active_preset());
    if (options_err != ESP_OK)
        ESP_LOGW(TAG, "Active preset options load failed: %s",
                 esp_err_to_name(options_err));

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
            .uri = "/api/dsp/loudness",
            .method = HTTP_POST,
            .handler = api_loudness_handler
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