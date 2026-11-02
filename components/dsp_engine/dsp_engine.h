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
 * Bass Harmonic Exciter
 * ============================================================ */

#define EXC_H2_MIN              0.00f
#define EXC_H2_MAX              0.12f

#define EXC_H3_MIN              0.00f
#define EXC_H3_MAX              0.06f

#define EXC_CROSSOVER_MIN       30.0f
#define EXC_CROSSOVER_MAX       100.0f

typedef struct {
    bool  enabled;

    /* Internal representation:
     * 0.06 = 6%
     * 0.02 = 2%
     */
    float h2_pct;

    float h3_pct;

    float crossover_hz;
} dsp_exciter_t;

/* ============================================================
 * Analog Character diagnostic bypass masks (session-only)
 * ============================================================ */

#define DSP_CHARACTER_STAGE_1          (1U << 0)
#define DSP_CHARACTER_STAGE_2          (1U << 1)
#define DSP_CHARACTER_STAGE_3          (1U << 2)
#define DSP_CHARACTER_STAGE_MASK_ALL   (DSP_CHARACTER_STAGE_1 | DSP_CHARACTER_STAGE_2 | DSP_CHARACTER_STAGE_3)

#define DSP_CHARACTER_FEATURE_WARMTH   (1U << 0)
#define DSP_CHARACTER_FEATURE_DETAIL   (1U << 1)
#define DSP_CHARACTER_FEATURE_DRIVE    (1U << 2)
#define DSP_CHARACTER_FEATURE_SOFTNESS (1U << 3)
#define DSP_CHARACTER_FEATURE_AIR      (1U << 4)
#define DSP_CHARACTER_FEATURE_MEMORY   (1U << 5)
#define DSP_CHARACTER_FEATURE_SPECTRAL (1U << 6)
#define DSP_CHARACTER_FEATURE_MASK_ALL ((1U << 7) - 1U)

typedef struct {
    uint8_t stage_mask;
    uint8_t feature_mask;
    uint32_t blocks;
    uint32_t stage_avg_us[3];
    uint32_t stage_max_us[3];
    uint32_t cpu_mhz;
} dsp_character_diagnostics_t;

/* Diagnostic masks are volatile and are not written to NVS. */
esp_err_t dsp_engine_set_character_diagnostics(uint8_t stage_mask,
                                               uint8_t feature_mask);
esp_err_t dsp_engine_get_character_diagnostics(dsp_character_diagnostics_t *out);

/* ============================================================
 * Analog Character Engine V4.1
 * Three-stage asymmetric polynomial waveshaper + exact first-order ADAA + state memory.
 * Public structure/API remains V2-compatible; no NVS layout change.
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
    float softness_pct; /* 0..100: backs off harmonics on loud peaks */
    float air_pct;      /* 0..100: high-frequency shelf, up to +1.5 dB */
    float memory_pct;   /* 0..100: bias/coupling/shared-supply memory depth */
    float spectral_pct;/* 0..100: low-to-upper-stage saturation balance */
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

/* ============================================================
 * Bass Harmonic Exciter
 * ============================================================ */

esp_err_t dsp_engine_set_exciter(bool enabled,
                                 float h2_pct,
                                 float h3_pct,
                                 float crossover_hz);

esp_err_t dsp_engine_get_exciter(dsp_exciter_t *out);

esp_err_t dsp_engine_set_exciter_enabled(bool enabled);
esp_err_t dsp_engine_set_exciter_h2(float h2_pct);
esp_err_t dsp_engine_set_exciter_h3(float h3_pct);
esp_err_t dsp_engine_set_exciter_crossover(float crossover_hz);

esp_err_t dsp_engine_save_exciter(void);

/* Analog Character */
esp_err_t dsp_engine_set_character(const dsp_character_t *config);
esp_err_t dsp_engine_get_character(dsp_character_t *out);
esp_err_t dsp_engine_set_character_profile(dsp_character_mode_t mode);
esp_err_t dsp_engine_save_character(void);

#ifdef __cplusplus
}
#endif