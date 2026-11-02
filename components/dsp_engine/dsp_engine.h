#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * DSP limits
 * ============================================================ */

#define DSP_MAX_BANDS          10
#define DSP_NUM_STAGES         (DSP_MAX_BANDS + 1)
#define DSP_NUM_PRESETS        4

/* ============================================================
 * Presets
 * ============================================================ */

typedef enum {
    DSP_PRESET_BEO = 0,
    DSP_PRESET_DEVIALET,
    DSP_PRESET_BOSE,
    DSP_PRESET_DIY_PURE,
    DSP_PRESET_MAX
} dsp_preset_id_t;

/* One parametric EQ band */
typedef struct {
    float freq;
    float gain_db;
    float q;
} dsp_band_t;

/* Complete preset */
typedef struct {
    char     name[16];

    uint8_t  num_bands;

    dsp_band_t bands[DSP_MAX_BANDS];

    float shelf_freq;
    float shelf_gain_db;

    float master_gain_db;
} dsp_preset_t;

/* ============================================================
 * Analog Character diagnostic bypass masks (session-only)
 * ============================================================ */

#define DSP_CHARACTER_STAGE_1          (1U << 0)
#define DSP_CHARACTER_STAGE_2          (1U << 1)
#define DSP_CHARACTER_STAGE_3          (1U << 2) /* Reserved; bypassed in V5 Lite. */
#define DSP_CHARACTER_STAGE_MASK_ALL   (DSP_CHARACTER_STAGE_1 | DSP_CHARACTER_STAGE_2)

#define DSP_CHARACTER_FEATURE_WARMTH   (1U << 0)
#define DSP_CHARACTER_FEATURE_DETAIL   (1U << 1)
#define DSP_CHARACTER_FEATURE_DRIVE    (1U << 2)
/* V4-only feature bits retained for source/NVS compatibility; ignored in V5. */
#define DSP_CHARACTER_FEATURE_SOFTNESS (1U << 3)
#define DSP_CHARACTER_FEATURE_AIR      (1U << 4)
#define DSP_CHARACTER_FEATURE_MEMORY   (1U << 5)
#define DSP_CHARACTER_FEATURE_SPECTRAL (1U << 6)
#define DSP_CHARACTER_FEATURE_MASK_ALL (DSP_CHARACTER_FEATURE_WARMTH | DSP_CHARACTER_FEATURE_DETAIL | DSP_CHARACTER_FEATURE_DRIVE)

typedef struct {
    uint8_t stage_mask;
    uint8_t feature_mask;
    uint32_t blocks;
    uint32_t stage_avg_us[3]; /* Stage 3 slot is reserved and remains zero in V5. */
    uint32_t stage_max_us[3];
    uint32_t cpu_mhz;
} dsp_character_diagnostics_t;

/* Diagnostic masks are volatile and are not written to NVS. */
esp_err_t dsp_engine_set_character_diagnostics(uint8_t stage_mask,
                                               uint8_t feature_mask);
esp_err_t dsp_engine_get_character_diagnostics(dsp_character_diagnostics_t *out);

/* ============================================================
 * Analog Character Engine V5.1 Lite
 * Two-stage polynomial waveshaper + mean-square tracking and optional controls.
 * V5.1 adds versioned NVS migration for the appended controls.
 * ============================================================ */

typedef enum {
    DSP_CHARACTER_CUSTOM  = 0,
    DSP_CHARACTER_TUBE    = 1, /* A: warm / even-harmonic body */
    DSP_CHARACTER_BRITISH = 2  /* B: natural / open / restrained */
} dsp_character_mode_t;

typedef struct {
    bool enabled;
    dsp_character_mode_t mode;
    float warmth_pct;   /* 0..100: normalized H2/body amount */
    float detail_pct;   /* 0..100: normalized H3/detail amount */
    float drive_pct;    /* 0..100: overall harmonic mix multiplier */
    float softness_pct; /* Reserved V4 field; ignored by V5 Lite. */
    float air_pct;      /* Reserved V4 field; ignored by V5 Lite. */
    float memory_pct;   /* Reserved V4 field; ignored by V5 Lite. */
    float spectral_pct; /* Reserved V4 field; ignored by V5 Lite. */

    /* V5.1 Lite controls; defaults are OFF to preserve V5 Lite voicing. */
    bool harmonic_interaction_enabled;
    float harmonic_interaction_pct; /* 0..100: Stage 1 -> Stage 2 coupling */
    bool dynamic_saturation_enabled;
    float dynamic_saturation_pct;   /* 0..100: level-dependent harmonic drive */
} dsp_character_t;

/* ============================================================
 * Engine
 * ============================================================ */

esp_err_t dsp_engine_init(void);

/* Process stereo PCM16.
 *
 * pcm_in   : interleaved stereo PCM16
 * len_bytes: input size in bytes
 * out_len  : output size in bytes
 *
 * Return:
 *   input pointer when bypassed
 *   internal scratch buffer when DSP is active
 */
const uint8_t *dsp_engine_process(const uint8_t *pcm_in,
                                  size_t len_bytes,
                                  size_t *out_len);

/* ============================================================
 * DSP enable
 * ============================================================ */

esp_err_t dsp_engine_set_enabled(bool enabled);
bool      dsp_engine_get_enabled(void);

/* ============================================================
 * Active preset
 * ============================================================ */

/* Original API */
esp_err_t       dsp_engine_select_preset(dsp_preset_id_t id);
dsp_preset_id_t dsp_engine_get_active_preset(void);

/* Compatibility API used by dsp_api.c */
esp_err_t dsp_engine_set_active(dsp_preset_id_t id);
dsp_preset_id_t dsp_engine_get_active(void);
esp_err_t dsp_engine_save_active(void);

/* ============================================================
 * Preset editing
 * ============================================================ */

esp_err_t dsp_engine_set_band(dsp_preset_id_t id,
                              uint8_t band_idx,
                              float freq,
                              float gain_db,
                              float q);

esp_err_t dsp_engine_set_shelf(dsp_preset_id_t id,
                               float freq,
                               float gain_db);

esp_err_t dsp_engine_set_master(dsp_preset_id_t id,
                                float gain_db);

esp_err_t dsp_engine_get_preset(dsp_preset_id_t id,
                                dsp_preset_t *out);

const char *dsp_engine_preset_name(dsp_preset_id_t id);

/* ============================================================
 * Preset persistence
 * ============================================================ */

esp_err_t dsp_engine_save_preset(dsp_preset_id_t id);
esp_err_t dsp_engine_reset_preset(dsp_preset_id_t id);

/* Analog Character */
esp_err_t dsp_engine_set_character(const dsp_character_t *config);
esp_err_t dsp_engine_get_character(dsp_character_t *out);
esp_err_t dsp_engine_set_character_profile(dsp_character_mode_t mode);
esp_err_t dsp_engine_save_character(void);

#ifdef __cplusplus
}
#endif