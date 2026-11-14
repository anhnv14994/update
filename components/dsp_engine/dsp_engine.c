#include "dsp_engine.h"
#include "esp_attr.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdatomic.h>

#include "esp_cpu.h"
#include "sdkconfig.h"

#if CONFIG_DSP_PEQ_USE_ESP_DSP || CONFIG_DSP_PEQ_USE_ESP_DSP_AES3
#include "dsps_biquad.h"
#endif

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


static const char *TAG = "dsp_engine";


/* ============================================================
 * Constants
 * ============================================================ */

#define DSP_SAMPLE_RATE_HZ   44100.0f

/* Fixed subsonic protection: 4th-order Butterworth HPF, always active. */
#define DSP_SUBSONIC_HPF_HZ       30.0f
#define DSP_SUBSONIC_HPF_Q1       0.541196100146197f
#define DSP_SUBSONIC_HPF_Q2       1.306562964876377f

#define DSP_NVS_NAMESPACE    "dsp_cfg"
#define DSP_NVS_KEY_ENABLED  "enabled"
#define DSP_NVS_KEY_ACTIVE   "active"
#define DSP_SCRATCH_MAX_BYTES 2048U

#define DSP_PRESET_MAGIC     0x44535031u   /* "DSP1" */
#define DSP_PRESET_VERSION   2u


#define DSP_CHARACTER_MAGIC   0x43485231u   /* "CHR1" */
#define DSP_CHARACTER_VERSION 3u
#define DSP_NVS_KEY_CHARACTER "character"


/* ============================================================
 * Biquad
 * ============================================================ */

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} biquad_coef_t;


typedef struct {
    float z1;
    float z2;
} biquad_state_t;


/* ============================================================
 * NVS blobs
 * ============================================================ */

typedef struct {
    uint32_t      magic;
    uint32_t      version;
    dsp_preset_t  preset;
} dsp_preset_blob_t;


typedef struct {
    uint32_t          magic;
    uint32_t          version;
    dsp_character_t   character;
} dsp_character_blob_t;

/* Exact V5 Lite / Character NVS V2 layout for backward migration. */
typedef struct {
    bool enabled;
    dsp_character_mode_t mode;
    float warmth_pct;
    float detail_pct;
    float drive_pct;
    float softness_pct;
    float air_pct;
    float memory_pct;
    float spectral_pct;
} dsp_character_v5_legacy_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    dsp_character_v5_legacy_t character;
} dsp_character_v5_legacy_blob_t;

/* Exact V3/V2 persisted layout: do not change this migration structure. */
typedef struct {
    bool enabled;
    dsp_character_mode_t mode;
    float warmth_pct;
    float detail_pct;
    float drive_pct;
    float softness_pct;
    float air_pct;
} dsp_character_v3_legacy_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    dsp_character_v3_legacy_t character;
} dsp_character_v3_legacy_blob_t;


/* ============================================================
 * Runtime state
 * ============================================================ */

static SemaphoreHandle_t s_lock = NULL;

static bool            s_enabled = true;
static dsp_preset_id_t s_active  = DSP_PRESET_BEO;

static dsp_preset_t s_presets[DSP_NUM_PRESETS];


/* ============================================================
 * Active PEQ chain
 * ============================================================ */

static uint8_t        s_active_num_stages = 0;
static biquad_coef_t  s_active_coefs[DSP_NUM_STAGES];

static float          s_active_master_linear = 1.0f;
static float          s_active_low_peak_eq_gain_db = 0.0f;
static float          s_active_high_peak_eq_gain_db = 0.0f;

#if CONFIG_DSP_PEQ_USE_ESP_DSP_AES3
/*
 * Experimental ESP32-S3 mono biquad path. Two planar channels allow
 * dsps_biquad_f32() to select the S3 AES3 implementation when the
 * resolved ESP-DSP package has it and optimized kernels are enabled.
 */
#define DSP_PEQ_MAX_FRAMES (DSP_SCRATCH_MAX_BYTES / (2U * sizeof(int16_t)))
static float s_peq_left[DSP_PEQ_MAX_FRAMES];
static float s_peq_right[DSP_PEQ_MAX_FRAMES];
static float s_peq_state[DSP_NUM_STAGES][2][2]; /* [stage][L/R][w0,w1] */
#elif CONFIG_DSP_PEQ_USE_ESP_DSP
/*
 * Experimental interleaved stereo float block + ESP-DSP SF32 kernel.
 * Holds the largest 2048-byte RAOP PCM block (512 stereo frames).
 */
#define DSP_PEQ_MAX_FLOATS (DSP_SCRATCH_MAX_BYTES / sizeof(int16_t))
static float s_peq_interleaved[DSP_PEQ_MAX_FLOATS];
static float s_peq_state[DSP_NUM_STAGES][4]; /* w0L,w1L,w0R,w1R */
#else
static biquad_state_t s_state[DSP_NUM_STAGES][2];
#endif

/* Two cascaded 2nd-order sections form a 4th-order Butterworth HPF. */
static biquad_coef_t  s_subsonic_hpf_coefs[2];
static biquad_state_t s_subsonic_hpf_state[2][2]; /* [L/R][section] */


/* ============================================================
 * Scratch buffer
 * ============================================================ */

static uint8_t *s_scratch = NULL;
static size_t   s_scratch_len = 0;


/* ============================================================
 * Analog Character runtime
 * ============================================================ */

static dsp_character_t s_character = {
    .enabled      = false, /* Preserve the existing sound after firmware update. */
    .mode         = DSP_CHARACTER_CUSTOM,
    .warmth_pct   = 0.0f,
    .detail_pct   = 0.0f,
    .drive_pct    = 0.0f,
    .softness_pct = 0.0f,
    .air_pct      = 0.0f,
};

#define CHARACTER_STAGE_COUNT 2

typedef struct {
    biquad_coef_t input_hpf;
    biquad_coef_t input_lpf;
    float ms_alpha;
} character_stage_filter_t;

typedef struct {
    character_stage_filter_t stage[CHARACTER_STAGE_COUNT];
    /* The identical residual HPF is shared after summing both stage outputs. */
    biquad_coef_t residual_hpf;
    /* HF source bands for transient A/B and the separate gated H2 Air path. */
    biquad_coef_t hf_band_a; /* 2–5 kHz, centered at 3.16 kHz */
    biquad_coef_t hf_band_b; /* 5–10 kHz, centered at 7.07 kHz */
    biquad_coef_t hf_air_hpf; /* Keep generated 2nd harmonic in the 4–10 kHz band. */
    float hf_air_alpha;
    float sag_attack_alpha;
    float sag_release_alpha;
    float hf_fast_attack_alpha;
    float hf_fast_release_alpha;
    float hf_slow_attack_alpha;
    float hf_slow_release_alpha;
    float hf_mix_alpha;
    float loudness_gain_alpha;
    float loudness_low_alpha;
    float loudness_high_alpha;
} character_filter_coef_t;

typedef struct {
    biquad_state_t input_hpf[2];
    biquad_state_t input_lpf[2];
    float mean_square[2];
    float prev_x[2];
    float prev_band_power[2];
    float prev_even_gain[2];
    float prev_odd_gain[2];
    float prev_mean_square[2];
    bool adaa_valid[2];
} character_stage_state_t;

typedef struct {
    character_stage_state_t stage[CHARACTER_STAGE_COUNT];
    float warmth;
    float detail;
    float drive;
    float interaction;
    float dynamic_saturation;
    float air_harmonic;
    float sag_envelope;
    float sag_gain;
    bool running;
} character_state_t;

typedef struct {
    float h2_gain[CHARACTER_STAGE_COUNT];
    float h3_gain[CHARACTER_STAGE_COUNT];
    float im_guard_amount;
    float dynamic_scale;
    float interaction_scale;
} character_gain_coeffs_t;


static character_filter_coef_t s_character_filters;
static character_state_t s_character_state = { .sag_gain = 1.0f };
static biquad_state_t s_character_residual_hpf[2];

/* Audio-task-owned state; no allocation, locks, or NVS writes in the PCM path. */
typedef struct {
    biquad_state_t band_a[2];
    biquad_state_t band_b[2];
    biquad_state_t air_hpf[2];
    float air_mean_square[2];
    float fast_env_a[2];
    float slow_env_a[2];
    float fast_env_b[2];
    float slow_env_b[2];
    float mix_a;
    float mix_b;
    bool history_active;
} character_hf_state_t;
static character_hf_state_t s_character_hf_state;
static _Atomic bool s_character_hf_a_enabled = false;
static _Atomic bool s_character_hf_b_enabled = false;
static _Atomic uint32_t s_character_hf_a_strength_pct = 0;
static _Atomic uint32_t s_character_hf_b_strength_pct = 0;

/* AirPlay volume is kept out of the PCM16 path until final output quantization. */
static _Atomic uint32_t s_output_volume_q = 50000U; /* preserve RAOP's 0.05 startup level */
static _Atomic bool s_dynamic_loudness_enabled = false;
static _Atomic uint32_t s_dynamic_bass_max_db = 4U; /* 0..5 dB, saved per preset */
static _Atomic uint32_t s_dynamic_reference_spl_db = 0U; /* 0 = estimated, not SPL calibrated */
static _Atomic uint32_t s_dynamic_reference_gain_q = 500000U; /* 0.5x effective gain */

typedef struct {
    float low_lp[2];
    float high_lp[2];
    float bass_gain;
    float treble_gain;
    float master_gain;
    bool filter_initialized[2];
} dsp_output_state_t;

static dsp_output_state_t s_output_state = {
    .bass_gain = 1.0f,
    .treble_gain = 1.0f,
    .master_gain = 1.0f,
};

/* Third diagnostic slot measures the optional HF refinement path. */
#define DSP_CHARACTER_DIAG_SLOTS 3U
/* A/B diagnostic masks are persisted per preset; timing counters remain runtime-only. */
static _Atomic uint32_t s_character_diag_stage_mask = DSP_CHARACTER_STAGE_MASK_ALL;
static _Atomic uint32_t s_character_diag_feature_mask = DSP_CHARACTER_FEATURE_MASK_ALL;
static _Atomic uint32_t s_character_diag_blocks = 0;
static _Atomic uint32_t s_character_diag_stage_total_us[DSP_CHARACTER_DIAG_SLOTS] = {0};
static _Atomic uint32_t s_character_diag_stage_max_us[DSP_CHARACTER_DIAG_SLOTS] = {0};

/* Audio-task-owned state used to clear stale histories after a test bypass. */
static uint8_t s_character_active_stage_mask = 0;
static _Atomic bool s_stream_reset_requested = false;


/* ============================================================
 * Factory presets
 *
 * DO NOT CHANGE THESE VALUES.
 * ============================================================ */

static const dsp_preset_t s_factory_presets[DSP_NUM_PRESETS] = {

    /* =========================================================
     * B&O — Vocal / Ballad
     * Vocal forward, smooth mid, controlled deep bass,
     * airy but non-harsh treble.
     * ========================================================= */
    [DSP_PRESET_BEO] = {
        .name = "B&O/Vocal",
        .num_bands = 10,

        .bands = {
            {   35.0f,  1.5f, 0.75f },
            {   70.0f,  1.0f, 1.00f },
            {  120.0f, -0.8f, 1.20f },
            {  250.0f, -1.2f, 1.40f },
            {  500.0f,  0.5f, 1.30f },
            { 1200.0f,  1.2f, 1.40f },
            { 2500.0f,  1.8f, 1.50f },
            { 4500.0f,  0.8f, 1.60f },
            { 8000.0f, -0.5f, 1.20f },
            { 14000.0f, 1.5f, 0.70f },
        },

        .shelf_freq = 16000.0f,
        .shelf_gain_db = 0.8f,
        .master_gain_db = -6.0f,
    },


    /* =========================================================
     * DEVIALET — EDM / Trance / Fast
     * Tight punch, deep controlled sub, fast kick/bass,
     * clean low-mid and energetic upper attack.
     * ========================================================= */
    [DSP_PRESET_DEVIALET] = {
        .name = "DEVIALET/TRANCE",
        .num_bands = 10,

        .bands = {
            {   32.0f,  2.5f, 0.85f },
            {   55.0f,  2.0f, 1.30f },
            {   90.0f,  1.5f, 1.70f },
            {  180.0f, -1.5f, 1.80f },
            {  350.0f, -1.0f, 1.50f },
            {  800.0f, -0.3f, 1.20f },
            { 2000.0f,  0.8f, 1.50f },
            { 4000.0f,  1.5f, 1.70f },
            { 8000.0f,  1.0f, 1.50f },
            { 15000.0f, 1.8f, 0.75f },
        },

        .shelf_freq = 16000.0f,
        .shelf_gain_db = 0.8f,
        .master_gain_db = -6.0f,
    },


    /* =========================================================
     * BOSE — EDM House / Warm / Vocal
     * Fuller bass, softer resonance, warm body,
     * vocal remains present above the bass.
     * ========================================================= */
    [DSP_PRESET_BOSE] = {
        .name = "BOSE/HOUSE",
        .num_bands = 10,

        .bands = {
            {   35.0f,  2.0f, 0.70f },
            {   60.0f,  2.5f, 0.90f },
            {  100.0f,  2.0f, 1.00f },
            {  180.0f,  0.5f, 1.20f },
            {  300.0f, -1.0f, 1.40f },
            {  700.0f,  0.2f, 1.20f },
            { 1800.0f,  1.5f, 1.30f },
            { 3200.0f,  1.2f, 1.50f },
            { 7000.0f, -0.5f, 1.20f },
            { 13000.0f, 1.0f, 0.70f },
        },

        .shelf_freq = 16000.0f,
        .shelf_gain_db = 0.5f,
        .master_gain_db = -6.0f,
    },


    /* =========================================================
     * DIY PURE — Instruments
     * Natural / neutral / low coloration.
     * Preserve timbre and harmonic structure of instruments.
     * ========================================================= */
    [DSP_PRESET_DIY_PURE] = {
        .name = "Musical",
        .num_bands = 10,

        .bands = {
            {   35.0f,  0.5f, 0.70f },
            {   70.0f,  0.5f, 0.90f },
            {  120.0f,  0.0f, 1.00f },
            {  250.0f, -0.5f, 1.20f },
            {  500.0f,  0.3f, 1.20f },
            { 1000.0f,  0.3f, 1.20f },
            { 2000.0f,  0.5f, 1.30f },
            { 4000.0f,  0.3f, 1.30f },
            { 8000.0f,  0.0f, 1.00f },
            { 14000.0f, 0.5f, 0.70f },
        },

        .shelf_freq = 16000.0f,
        .shelf_gain_db = 0.3f,
        .master_gain_db = -6.0f,
    },
};


/* ============================================================
 * Helpers
 * ============================================================ */

static inline float clampf_local(float x, float lo, float hi)
{
    if (x < lo)
        return lo;

    if (x > hi)
        return hi;

    return x;
}

/* Audio-task-owned PRNG: TPDF dither at the Float -> PCM16 boundary. */
static uint32_t s_dsp_dither_rng = 0xA341316CU;

static inline float dsp_tpdf_dither_lsb(void)
{
    s_dsp_dither_rng = s_dsp_dither_rng * 1664525U + 1013904223U;
    const float a = (float)(s_dsp_dither_rng >> 8) * (1.0f / 16777216.0f);
    s_dsp_dither_rng = s_dsp_dither_rng * 1664525U + 1013904223U;
    const float b = (float)(s_dsp_dither_rng >> 8) * (1.0f / 16777216.0f);
    return a - b;
}

static inline int16_t float_to_pcm16_rounded(float sample)
{
    if (!isfinite(sample))
        return 0;

    /* Do not dither values that are far below one PCM16 quantization step. */
    if (fabsf(sample) < 1.0e-7f)
        return 0;

    /* One TPDF-LSB peak dither before this PCM16 quantization boundary. */
    const float scaled = clampf_local(
        sample * 32768.0f + dsp_tpdf_dither_lsb(),
        -32768.0f, 32767.0f);
    const int32_t rounded = (int32_t)(
        scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
    return (int16_t)rounded;
}

/* Derive all control-dependent Character coefficients once per PCM block. */
static inline void character_calculate_gain_coeffs(
    float warmth,
    float detail,
    float drive_control,
    float interaction,
    float dynamic_saturation,
    bool drive_enabled,
    character_gain_coeffs_t *out)
{
    const float drive = drive_enabled
        ? (0.90f + 1.90f * drive_control) : 1.0f;

    const float stage_bias_0 = 0.020f + 0.105f * warmth;
    const float stage_bias_1 = 0.010f + 0.065f * warmth + 0.018f * detail;
    const float asymmetry_0 = clampf_local(
        0.62f + 1.35f * stage_bias_0, 0.35f, 0.95f);
    const float asymmetry_1 = clampf_local(
        0.62f + 1.35f * stage_bias_1, 0.35f, 0.95f);

    out->h2_gain[0] = 0.190f * warmth * drive * asymmetry_0;
    out->h2_gain[1] = 0.105f * warmth * drive * asymmetry_1;
    out->h3_gain[0] = 0.0f;
    /* A modest +3.5 dB sensitivity increase makes Detail useful at normal settings. */
    out->h3_gain[1] = 0.075f * detail * drive;

    const float diagnostic_drive = drive_enabled ? drive_control : 0.0f;
    const float harmonic_intensity = clampf_local(
        0.65f * warmth + 0.35f * diagnostic_drive, 0.0f, 1.0f);

    out->im_guard_amount = 0.22f * harmonic_intensity;
    out->dynamic_scale = 0.20f * dynamic_saturation;
    out->interaction_scale = 0.25f * interaction;
}


static inline float db_to_linear(float db)
{
    return powf(10.0f, db / 20.0f);
}

/*
 * ISO 226:2023 equal-loudness contour, evaluated at the two adaptive bands.
 * The 110 Hz and 11 kHz coefficients are log-frequency interpolations of
 * the standard's 100/125 Hz and 10/12.5 kHz rows.
 */
static inline float iso226_spl_db(
    float phon, float alpha_f, float lu_db, float threshold_db)
{
    phon = clampf_local(phon, 20.0f, 80.0f);
    const float loudness_term =
        powf(10.0f, 0.03f * phon) - powf(10.0f, 0.072f);
    const float transfer_term =
        powf(4.0e-10f, 0.3f - alpha_f) * loudness_term;
    const float threshold_term =
        powf(10.0f, alpha_f * (threshold_db + lu_db) * 0.1f);
    const float total = fmaxf(transfer_term + threshold_term, 1.0e-20f);
    return (10.0f / alpha_f) * log10f(total) - lu_db;
}

static inline float iso226_relative_compensation_db(
    float current_phon, float reference_phon,
    float alpha_f, float lu_db, float threshold_db)
{
    const float current_delta =
        iso226_spl_db(current_phon, alpha_f, lu_db, threshold_db) -
        clampf_local(current_phon, 20.0f, 80.0f);
    const float reference_delta =
        iso226_spl_db(reference_phon, alpha_f, lu_db, threshold_db) -
        clampf_local(reference_phon, 20.0f, 80.0f);
    return current_delta - reference_delta;
}

/* Smooth master trim per sample; about an 8 ms time constant at 44.1 kHz. */
static inline float smooth_master_gain(float current, float target)
{
    current += 0.002830453811f * (target - current);
    if (fabsf(target - current) < 1.0e-6f)
        current = target;
    return current;
}

/* Evaluate the cached PEQ+shelf magnitude for one frequency. */
static float biquad_response_db(const biquad_coef_t *c, float freq_hz)
{
    const float w = 2.0f * (float)M_PI * freq_hz / DSP_SAMPLE_RATE_HZ;
    const float c1 = cosf(w);
    const float s1 = sinf(w);
    const float c2 = cosf(2.0f * w);
    const float s2 = sinf(2.0f * w);

    const float nr = c->b0 + c->b1 * c1 + c->b2 * c2;
    const float ni = -c->b1 * s1 - c->b2 * s2;
    const float dr = 1.0f + c->a1 * c1 + c->a2 * c2;
    const float di = -c->a1 * s1 - c->a2 * s2;
    const float den = fmaxf(dr * dr + di * di, 1.0e-20f);
    const float mag2 = (nr * nr + ni * ni) / den;

    return 10.0f * log10f(fmaxf(mag2, 1.0e-12f));
}

/*
 * Conservative digital headroom estimate for Dynamic Loudness.
 * Evaluate the PEQ+shelf chain over bass and treble bands when a preset
 * is rebuilt, not in the real-time sample loop. Only positive boost is
 * returned; cuts do not create extra loudness compensation.
 */
static void estimate_eq_headroom_db(
    const biquad_coef_t *coefs,
    uint8_t num_stages,
    float *low_peak_db,
    float *high_peak_db)
{
    float low_peak = 0.0f;
    float high_peak = 0.0f;

    for (unsigned i = 0; i < 64U; ++i) {
        const float t = (float)i / 63.0f;
        const float low_hz = 20.0f * powf(250.0f / 20.0f, t);
        const float high_hz = 6000.0f * powf(20000.0f / 6000.0f, t);
        float low_db = 0.0f;
        float high_db = 0.0f;

        for (uint8_t stage = 0; stage < num_stages; ++stage) {
            low_db += biquad_response_db(&coefs[stage], low_hz);
            high_db += biquad_response_db(&coefs[stage], high_hz);
        }

        if (low_db > low_peak)
            low_peak = low_db;
        if (high_db > high_peak)
            high_peak = high_db;
    }

    if (low_peak_db)
        *low_peak_db = low_peak;
    if (high_peak_db)
        *high_peak_db = high_peak;
}


/* ============================================================
 * Generic Biquad
 *
 * Direct Form II Transposed
 * ============================================================ */

static inline float biquad_process_sample(
    float x,
    const biquad_coef_t *c,
    biquad_state_t *s)
{
    float y =
        c->b0 * x +
        s->z1;

    s->z1 =
        c->b1 * x -
        c->a1 * y +
        s->z2;

    s->z2 =
        c->b2 * x -
        c->a2 * y;

    return y;
}


/* ============================================================
 * RBJ Peaking EQ
 * ============================================================ */

static void calc_peaking(float fs,
                         float f0,
                         float gain_db,
                         float q,
                         biquad_coef_t *out)
{
    f0 = clampf_local(f0, 10.0f, fs * 0.49f);
    q  = fmaxf(q, 0.1f);

    const float A =
        powf(10.0f, gain_db / 40.0f);

    const float w0 =
        2.0f * (float)M_PI * f0 / fs;

    const float cw = cosf(w0);
    const float sw = sinf(w0);

    const float alpha =
        sw / (2.0f * q);

    const float b0 =
        1.0f + alpha * A;

    const float b1 =
        -2.0f * cw;

    const float b2 =
        1.0f - alpha * A;

    const float a0 =
        1.0f + alpha / A;

    const float a1 =
        -2.0f * cw;

    const float a2 =
        1.0f - alpha / A;

    out->b0 = b0 / a0;
    out->b1 = b1 / a0;
    out->b2 = b2 / a0;

    out->a1 = a1 / a0;
    out->a2 = a2 / a0;
}


/* ============================================================
 * RBJ High Shelf
 * ============================================================ */

static void calc_high_shelf(float fs,
                            float f0,
                            float gain_db,
                            biquad_coef_t *out)
{
    f0 = clampf_local(
        f0,
        100.0f,
        fs * 0.49f
    );

    const float A =
        powf(10.0f, gain_db / 40.0f);

    const float w0 =
        2.0f * (float)M_PI * f0 / fs;

    const float cw = cosf(w0);
    const float sw = sinf(w0);

    const float S = 1.0f;

    const float alpha =
        sw / 2.0f *
        sqrtf(
            (A + 1.0f / A) *
            (1.0f / S - 1.0f)
            + 2.0f
        );

    const float beta =
        2.0f * sqrtf(A) * alpha;

    const float b0 =
        A * (
            (A + 1.0f)
            + (A - 1.0f) * cw
            + beta
        );

    const float b1 =
        -2.0f * A * (
            (A - 1.0f)
            + (A + 1.0f) * cw
        );

    const float b2 =
        A * (
            (A + 1.0f)
            + (A - 1.0f) * cw
            - beta
        );

    const float a0 =
        (A + 1.0f)
        - (A - 1.0f) * cw
        + beta;

    const float a1 =
        2.0f * (
            (A - 1.0f)
            - (A + 1.0f) * cw
        );

    const float a2 =
        (A + 1.0f)
        - (A - 1.0f) * cw
        - beta;

    out->b0 = b0 / a0;
    out->b1 = b1 / a0;
    out->b2 = b2 / a0;

    out->a1 = a1 / a0;
    out->a2 = a2 / a0;
}


/* ============================================================
 * Butterworth 2nd-order LPF
 * ============================================================ */

static void calc_butterworth_lpf_q(
    float fs,
    float freq,
    float q,
    biquad_coef_t *out)
{
    freq =
        clampf_local(
            freq,
            5.0f,
            fs * 0.45f
        );

    const float w0 =
        2.0f * (float)M_PI * freq / fs;

    const float cw = cosf(w0);
    const float sw = sinf(w0);

    q = fmaxf(q, 0.5f);

    const float alpha =
        sw / (2.0f * q);

    const float b0 =
        (1.0f - cw) * 0.5f;

    const float b1 =
        1.0f - cw;

    const float b2 =
        (1.0f - cw) * 0.5f;

    const float a0 =
        1.0f + alpha;

    const float a1 =
        -2.0f * cw;

    const float a2 =
        1.0f - alpha;

    out->b0 = b0 / a0;
    out->b1 = b1 / a0;
    out->b2 = b2 / a0;

    out->a1 = a1 / a0;
    out->a2 = a2 / a0;
}


/* ============================================================
 * Butterworth 2nd-order HPF
 * ============================================================ */

static void calc_butterworth_hpf_q(
    float fs,
    float freq,
    float q,
    biquad_coef_t *out)
{
    freq =
        clampf_local(
            freq,
            5.0f,
            fs * 0.45f
        );

    const float w0 =
        2.0f * (float)M_PI * freq / fs;

    const float cw = cosf(w0);
    const float sw = sinf(w0);

    q = fmaxf(q, 0.5f);

    const float alpha =
        sw / (2.0f * q);

    const float b0 =
        (1.0f + cw) * 0.5f;

    const float b1 =
        -(1.0f + cw);

    const float b2 =
        (1.0f + cw) * 0.5f;

    const float a0 =
        1.0f + alpha;

    const float a1 =
        -2.0f * cw;

    const float a2 =
        1.0f - alpha;

    out->b0 = b0 / a0;
    out->b1 = b1 / a0;
    out->b2 = b2 / a0;

    out->a1 = a1 / a0;
    out->a2 = a2 / a0;
}


/* ============================================================
 * Fixed subsonic HPF — 4th-order Butterworth
 * ============================================================ */
static void init_subsonic_hpf(void)
{
    calc_butterworth_hpf_q(
        DSP_SAMPLE_RATE_HZ,
        DSP_SUBSONIC_HPF_HZ,
        DSP_SUBSONIC_HPF_Q1,
        &s_subsonic_hpf_coefs[0]
    );
    calc_butterworth_hpf_q(
        DSP_SAMPLE_RATE_HZ,
        DSP_SUBSONIC_HPF_HZ,
        DSP_SUBSONIC_HPF_Q2,
        &s_subsonic_hpf_coefs[1]
    );

    memset(s_subsonic_hpf_state, 0, sizeof(s_subsonic_hpf_state));
}

static inline float subsonic_hpf_process_sample(float x, unsigned channel)
{
    x = biquad_process_sample(
        x, &s_subsonic_hpf_coefs[0],
        &s_subsonic_hpf_state[channel][0]);
    return biquad_process_sample(
        x, &s_subsonic_hpf_coefs[1],
        &s_subsonic_hpf_state[channel][1]);
}


/* ============================================================
 * Analog Character V5.1 Lite — two-stage low-cost ADAA
 * ============================================================

 * Each stage uses a band-limited source and an asymmetric quadratic/cubic
 * transfer. First-order ADAA is evaluated in closed form: no LUT access,
 * logarithm, or per-sample division. A lightweight mean-square detector adds
 * level-dependent harmonic shaping; V4 memory/supply branches are removed.
 *
 * For f(x)=a2*x^2+a3*x^3, the exact first-order ADAA interval averages are:
 *   AA2 = (x^2 + x*p + p^2) / 3
 *   AA3 = (x^3 + x^2*p + x*p^2 + p^3) / 4
 */
static inline float character_adaa_even_residual(
    float x, float x2, float x_prev, float x2_prev,
    float mean_square, float mean_square_prev,
    float even_gain)
{
    const float xp = x * x_prev;
    const float ms = 0.5f * (mean_square + mean_square_prev);
    const float aa2 = (x2 + xp + x2_prev) * 0.333333333333f;
    return even_gain * (aa2 - ms);
}

static inline float character_adaa_poly_residual(
    float x, float x2, float x_prev, float x2_prev,
    float mean_square, float mean_square_prev,
    float even_gain, float odd_gain)
{
    /* Reuse the detector's current power and cached previous power. */
    const float xp = x * x_prev;
    const float ms = 0.5f * (mean_square + mean_square_prev);
    float residual = 0.0f;

    if (even_gain != 0.0f) {
        const float aa2 = (x2 + xp + x2_prev) * 0.333333333333f;
        residual += even_gain * (aa2 - ms);
    }

    if (odd_gain != 0.0f) {
        /* Algebraically identical to x³+x²p+xp²+p³, with fewer multiplies. */
        const float aa3 = (x + x_prev) * (x2 + x2_prev) * 0.25f;
        const float mid = (x + x_prev) * 0.5f;
        residual += odd_gain * (aa3 - 1.5f * ms * mid);
    }

    return residual;
}

static void calc_bandpass(float fs, float center_hz, float q, biquad_coef_t *out)
{
    const float w0 = 2.0f * (float)M_PI * center_hz / fs;
    const float cw = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * q);
    const float a0 = 1.0f + alpha;

    out->b0 = alpha / a0;
    out->b1 = 0.0f;
    out->b2 = -alpha / a0;
    out->a1 = -2.0f * cw / a0;
    out->a2 = (1.0f - alpha) / a0;
}

static float envelope_alpha(float time_ms)
{
    return 1.0f - expf(-1.0f / (time_ms * 0.001f * DSP_SAMPLE_RATE_HZ));
}

static void rebuild_character_filters(void)
{
    character_filter_coef_t next;
    memset(&next, 0, sizeof(next));

    /* V6.7: controlled harmonic source-band overlap balances body and vocal core.
     * Stage 1 body: 80–450 Hz; Stage 2 vocal core: 250–1400 Hz.
     * Each path remains HPF2 + LPF2; no additional biquad cascade is added. */
    static const float source_hp[CHARACTER_STAGE_COUNT] = { 80.0f, 250.0f };
    static const float source_lp[CHARACTER_STAGE_COUNT] = { 450.0f, 1400.0f };
    static const float mean_square_ms[CHARACTER_STAGE_COUNT] = { 20.0f, 18.0f };

    for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
        calc_butterworth_hpf_q(DSP_SAMPLE_RATE_HZ, source_hp[stage],
                               0.70710678118f, &next.stage[stage].input_hpf);
        calc_butterworth_lpf_q(DSP_SAMPLE_RATE_HZ, source_lp[stage],
                               0.70710678118f, &next.stage[stage].input_lpf);
        next.stage[stage].ms_alpha = 1.0f - expf(
            -1.0f / (mean_square_ms[stage] * 0.001f * DSP_SAMPLE_RATE_HZ));
    }

    calc_butterworth_hpf_q(DSP_SAMPLE_RATE_HZ, 15.0f,
                           0.70710678118f, &next.residual_hpf);

    /* HF refinement: transient A/B plus a separate gated H2 Air path; no static treble shelf. */
    calc_bandpass(DSP_SAMPLE_RATE_HZ, 3162.2777f, 1.054f, &next.hf_band_a);
    calc_bandpass(DSP_SAMPLE_RATE_HZ, 7071.0678f, 1.414f, &next.hf_band_b);
    calc_butterworth_hpf_q(DSP_SAMPLE_RATE_HZ, 3500.0f,
                           0.70710678118f, &next.hf_air_hpf);
    next.hf_air_alpha = envelope_alpha(20.0f);
    next.sag_attack_alpha = envelope_alpha(5.0f);
    next.sag_release_alpha = envelope_alpha(150.0f);
    next.hf_fast_attack_alpha = envelope_alpha(1.5f);
    next.hf_fast_release_alpha = envelope_alpha(25.0f);
    next.hf_slow_attack_alpha = envelope_alpha(30.0f);
    next.hf_slow_release_alpha = envelope_alpha(120.0f);
    next.hf_mix_alpha = 1.0f - expf(
        -1.0f / (0.008f * DSP_SAMPLE_RATE_HZ));

    /* Loudness shelves settle in 50 ms. Final volume ramp lives in I2S. */
    next.loudness_gain_alpha = 1.0f - expf(
        -1.0f / (0.050f * DSP_SAMPLE_RATE_HZ));
    next.loudness_low_alpha = 1.0f - expf(
        -2.0f * (float)M_PI * 110.0f / DSP_SAMPLE_RATE_HZ);
    next.loudness_high_alpha = 1.0f - expf(
        -2.0f * (float)M_PI * 11000.0f / DSP_SAMPLE_RATE_HZ);

    if (s_lock)
        xSemaphoreTake(s_lock, portMAX_DELAY);
    s_character_filters = next;
    if (s_lock)
        xSemaphoreGive(s_lock);
}

static void reset_character_states(void)
{
    memset(&s_character_state, 0, sizeof(s_character_state));
    s_character_state.sag_gain = 1.0f;
    memset(s_character_residual_hpf, 0, sizeof(s_character_residual_hpf));
    memset(&s_character_hf_state, 0, sizeof(s_character_hf_state));
    s_character_active_stage_mask = 0;
}

static void reset_stream_processing_state(void)
{
#if CONFIG_DSP_PEQ_USE_ESP_DSP_AES3 || CONFIG_DSP_PEQ_USE_ESP_DSP
    memset(s_peq_state, 0, sizeof(s_peq_state));
#else
    memset(s_state, 0, sizeof(s_state));
#endif
    memset(s_subsonic_hpf_state, 0, sizeof(s_subsonic_hpf_state));
    reset_character_states();
    memset(&s_output_state, 0, sizeof(s_output_state));
    s_output_state.bass_gain = 1.0f;
    s_output_state.treble_gain = 1.0f;
    s_output_state.master_gain = 1.0f;
}

static void set_character_factory(void)
{
    memset(&s_character, 0, sizeof(s_character));
    s_character.mode = DSP_CHARACTER_CUSTOM;
}

static const char *character_switch_text(bool enabled)
{
    return enabled ? "ON" : "OFF";
}

/* Log only control transitions; never called from the real-time audio path. */
static void log_character_switch_changes(
    const char *source,
    bool old_dsp_enabled,
    const dsp_character_t *old_cfg,
    bool new_dsp_enabled,
    const dsp_character_t *new_cfg)
{
    const bool old_character_effective =
        old_dsp_enabled && old_cfg->enabled;
    const bool new_character_effective =
        new_dsp_enabled && new_cfg->enabled;

    if (old_cfg->enabled != new_cfg->enabled ||
        old_character_effective != new_character_effective) {
        ESP_LOGI(TAG,
                 "[%s] Analog Character: requested %s -> %s, effective %s -> %s",
                 source,
                 character_switch_text(old_cfg->enabled),
                 character_switch_text(new_cfg->enabled),
                 character_switch_text(old_character_effective),
                 character_switch_text(new_character_effective));
    }

    const bool old_interaction_effective =
        old_character_effective && old_cfg->harmonic_interaction_enabled;
    const bool new_interaction_effective =
        new_character_effective && new_cfg->harmonic_interaction_enabled;

    if (old_cfg->harmonic_interaction_enabled !=
            new_cfg->harmonic_interaction_enabled ||
        old_interaction_effective != new_interaction_effective) {
        ESP_LOGI(TAG,
                 "[%s] Harmonic Interaction: requested %s -> %s, effective %s -> %s, amount=%.0f%%",
                 source,
                 character_switch_text(old_cfg->harmonic_interaction_enabled),
                 character_switch_text(new_cfg->harmonic_interaction_enabled),
                 character_switch_text(old_interaction_effective),
                 character_switch_text(new_interaction_effective),
                 new_cfg->harmonic_interaction_pct);
    }

    const bool old_dynamic_effective =
        old_character_effective && old_cfg->dynamic_saturation_enabled;
    const bool new_dynamic_effective =
        new_character_effective && new_cfg->dynamic_saturation_enabled;

    if (old_cfg->dynamic_saturation_enabled !=
            new_cfg->dynamic_saturation_enabled ||
        old_dynamic_effective != new_dynamic_effective) {
        ESP_LOGI(TAG,
                 "[%s] Dynamic Saturation: requested %s -> %s, effective %s -> %s, amount=%.0f%%",
                 source,
                 character_switch_text(old_cfg->dynamic_saturation_enabled),
                 character_switch_text(new_cfg->dynamic_saturation_enabled),
                 character_switch_text(old_dynamic_effective),
                 character_switch_text(new_dynamic_effective),
                 new_cfg->dynamic_saturation_pct);
    }
}

static void log_character_switch_boot_state(bool dsp_enabled,
                                            const dsp_character_t *cfg)
{
    const bool character_effective = dsp_enabled && cfg->enabled;

    ESP_LOGI(TAG,
             "[boot/NVS] Analog Character: requested=%s effective=%s",
             character_switch_text(cfg->enabled),
             character_switch_text(character_effective));
    ESP_LOGI(TAG,
             "[boot/NVS] Harmonic Interaction: requested=%s effective=%s amount=%.0f%%",
             character_switch_text(cfg->harmonic_interaction_enabled),
             character_switch_text(character_effective &&
                                   cfg->harmonic_interaction_enabled),
             cfg->harmonic_interaction_pct);
    ESP_LOGI(TAG,
             "[boot/NVS] Dynamic Saturation: requested=%s effective=%s amount=%.0f%%",
             character_switch_text(cfg->dynamic_saturation_enabled),
             character_switch_text(character_effective &&
                                   cfg->dynamic_saturation_enabled),
             cfg->dynamic_saturation_pct);
}

static bool character_config_valid(const dsp_character_t *c)
{
    if (!c || c->mode < DSP_CHARACTER_CUSTOM ||
        c->mode > DSP_CHARACTER_BRITISH)
        return false;
    if (!isfinite(c->warmth_pct) || !isfinite(c->detail_pct) ||
        !isfinite(c->drive_pct) || !isfinite(c->softness_pct) ||
        !isfinite(c->air_pct) || !isfinite(c->memory_pct) ||
        !isfinite(c->spectral_pct) ||
        !isfinite(c->harmonic_interaction_pct) ||
        !isfinite(c->dynamic_saturation_pct))
        return false;
    return c->warmth_pct >= 0.0f && c->warmth_pct <= 100.0f &&
           c->detail_pct >= 0.0f && c->detail_pct <= 100.0f &&
           c->drive_pct >= 0.0f && c->drive_pct <= 100.0f &&
           c->softness_pct >= 0.0f && c->softness_pct <= 100.0f &&
           c->air_pct >= 0.0f && c->air_pct <= 100.0f &&
           c->memory_pct >= 0.0f && c->memory_pct <= 100.0f &&
           c->spectral_pct >= 0.0f && c->spectral_pct <= 100.0f &&
           c->harmonic_interaction_pct >= 0.0f &&
           c->harmonic_interaction_pct <= 100.0f &&
           c->dynamic_saturation_pct >= 0.0f &&
           c->dynamic_saturation_pct <= 100.0f;
}

static void character_profile_defaults(dsp_character_mode_t mode,
                                       dsp_character_t *out)
{
    if (!out)
        return;

    memset(out, 0, sizeof(*out));
    out->enabled = true;
    out->mode = mode;

    if (mode == DSP_CHARACTER_TUBE) {
        out->warmth_pct = 62.0f;
        out->detail_pct = 10.0f;
        out->drive_pct = 32.0f;
    } else {
        out->warmth_pct = 34.0f;
        out->detail_pct = 18.0f;
        out->drive_pct = 18.0f;
    }
}

static esp_err_t load_character_from_nvs(nvs_handle_t nvs)
{
    size_t len = 0;
    esp_err_t err = nvs_get_blob(nvs, DSP_NVS_KEY_CHARACTER, NULL, &len);
    if (err != ESP_OK) {
        set_character_factory();
        return ESP_OK;
    }

    if (len == sizeof(dsp_character_blob_t)) {
        dsp_character_blob_t blob;
        size_t read_len = sizeof(blob);
        err = nvs_get_blob(nvs, DSP_NVS_KEY_CHARACTER, &blob, &read_len);
        if (err == ESP_OK && read_len == sizeof(blob) &&
            blob.magic == DSP_CHARACTER_MAGIC &&
            blob.version == DSP_CHARACTER_VERSION &&
            character_config_valid(&blob.character)) {
            s_character = blob.character;
            s_character.softness_pct = 0.0f;
            /* air_pct is repurposed as the V6.9 optional HF H2 amount. */
            s_character.memory_pct = 0.0f;
            s_character.spectral_pct = 0.0f;
            return ESP_OK;
        }
    } else if (len == sizeof(dsp_character_v5_legacy_blob_t)) {
        dsp_character_v5_legacy_blob_t old_blob;
        size_t read_len = sizeof(old_blob);
        err = nvs_get_blob(nvs, DSP_NVS_KEY_CHARACTER, &old_blob, &read_len);
        if (err == ESP_OK && read_len == sizeof(old_blob) &&
            old_blob.magic == DSP_CHARACTER_MAGIC && old_blob.version == 2u) {
            memset(&s_character, 0, sizeof(s_character));
            s_character.enabled = old_blob.character.enabled;
            s_character.mode = old_blob.character.mode;
            s_character.warmth_pct = old_blob.character.warmth_pct;
            s_character.detail_pct = old_blob.character.detail_pct;
            s_character.drive_pct = old_blob.character.drive_pct;
            s_character.softness_pct = 0.0f;
            s_character.air_pct = 0.0f;
            s_character.memory_pct = 0.0f;
            s_character.spectral_pct = 0.0f;
            /* V5.1 additions default OFF, preserving the old V5 Lite sound. */
            s_character.harmonic_interaction_enabled = false;
            s_character.harmonic_interaction_pct = 0.0f;
            s_character.dynamic_saturation_enabled = false;
            s_character.dynamic_saturation_pct = 0.0f;

            if (character_config_valid(&s_character)) {
                dsp_character_blob_t migrated = {
                    .magic = DSP_CHARACTER_MAGIC,
                    .version = DSP_CHARACTER_VERSION,
                    .character = s_character
                };
                (void)nvs_set_blob(nvs, DSP_NVS_KEY_CHARACTER,
                                   &migrated, sizeof(migrated));
                ESP_LOGI(TAG, "Character V5 Lite NVS V2 migrated to V5.1 Lite");
                return ESP_OK;
            }
        }
    } else if (len == sizeof(dsp_character_v3_legacy_blob_t)) {
        dsp_character_v3_legacy_blob_t old_blob;
        size_t read_len = sizeof(old_blob);
        err = nvs_get_blob(nvs, DSP_NVS_KEY_CHARACTER, &old_blob, &read_len);
        if (err == ESP_OK && read_len == sizeof(old_blob) &&
            old_blob.magic == DSP_CHARACTER_MAGIC && old_blob.version == 1u) {
            memset(&s_character, 0, sizeof(s_character));
            s_character.enabled = old_blob.character.enabled;
            s_character.mode = old_blob.character.mode;
            s_character.warmth_pct = old_blob.character.warmth_pct;
            s_character.detail_pct = old_blob.character.detail_pct;
            s_character.drive_pct = old_blob.character.drive_pct;
            s_character.softness_pct = 0.0f;
            s_character.air_pct = 0.0f;
            s_character.memory_pct = 0.0f;
            s_character.spectral_pct = 0.0f;
            s_character.harmonic_interaction_enabled = false;
            s_character.harmonic_interaction_pct = 0.0f;
            s_character.dynamic_saturation_enabled = false;
            s_character.dynamic_saturation_pct = 0.0f;
            if (character_config_valid(&s_character)) {
                dsp_character_blob_t migrated = {
                    .magic = DSP_CHARACTER_MAGIC,
                    .version = DSP_CHARACTER_VERSION,
                    .character = s_character
                };
                (void)nvs_set_blob(nvs, DSP_NVS_KEY_CHARACTER,
                                   &migrated, sizeof(migrated));
                ESP_LOGI(TAG, "Character V3 NVS migrated to V5.1 Lite defaults");
                return ESP_OK;
            }
        }
    }

    ESP_LOGW(TAG, "Character NVS invalid/incompatible; using factory defaults");
    set_character_factory();
    return ESP_OK;
}


/* ============================================================
 * Apply preset
 * ============================================================ */

static bool dsp_preset_data_valid(const dsp_preset_t *preset)
{
    if (!preset || !memchr(preset->name, '\0', sizeof(preset->name)) ||
        preset->num_bands > DSP_MAX_BANDS)
        return false;

    if (!isfinite(preset->shelf_freq) ||
        !isfinite(preset->shelf_gain_db) ||
        !isfinite(preset->master_gain_db) ||
        preset->shelf_freq < 1000.0f || preset->shelf_freq > 20000.0f ||
        preset->shelf_gain_db < -12.0f || preset->shelf_gain_db > 12.0f ||
        preset->master_gain_db < -24.0f || preset->master_gain_db > 6.0f)
        return false;

    for (uint8_t i = 0; i < preset->num_bands; ++i) {
        const dsp_band_t *band = &preset->bands[i];
        if (!isfinite(band->freq) || !isfinite(band->gain_db) ||
            !isfinite(band->q) ||
            band->freq < 20.0f || band->freq > 20000.0f ||
            band->gain_db < -12.0f || band->gain_db > 12.0f ||
            band->q < 0.1f || band->q > 10.0f)
            return false;
    }

    return true;
}

static esp_err_t apply_preset_as_active(
    const dsp_preset_t *preset)
{
    if (!dsp_preset_data_valid(preset))
        return ESP_ERR_INVALID_ARG;

    biquad_coef_t new_coefs[DSP_NUM_STAGES];

    uint8_t num_bands =
        preset->num_bands;

    if (num_bands > DSP_MAX_BANDS)
        num_bands = DSP_MAX_BANDS;


    /*
     * PEQ bands.
     */

    for (uint8_t i = 0;
         i < num_bands;
         i++) {

        calc_peaking(
            DSP_SAMPLE_RATE_HZ,
            preset->bands[i].freq,
            preset->bands[i].gain_db,
            preset->bands[i].q,
            &new_coefs[i]
        );
    }


    /*
     * High shelf.
     */

    calc_high_shelf(
        DSP_SAMPLE_RATE_HZ,
        preset->shelf_freq,
        preset->shelf_gain_db,
        &new_coefs[num_bands]
    );


    const uint8_t num_stages =
        num_bands + 1;

    float low_peak_eq_gain_db = 0.0f;
    float high_peak_eq_gain_db = 0.0f;
    estimate_eq_headroom_db(
        new_coefs, num_stages,
        &low_peak_eq_gain_db, &high_peak_eq_gain_db);

    const float master_linear =
        db_to_linear(
            preset->master_gain_db
        );


    /*
     * Swap coefficients quickly.
     * Do not keep mutex during audio processing.
     */

    if (s_lock)
        xSemaphoreTake(
            s_lock,
            portMAX_DELAY
        );

    memcpy(
        s_active_coefs,
        new_coefs,
        sizeof(biquad_coef_t) * num_stages
    );

    s_active_num_stages =
        num_stages;

    s_active_master_linear =
        master_linear;
    s_active_low_peak_eq_gain_db = low_peak_eq_gain_db;
    s_active_high_peak_eq_gain_db = high_peak_eq_gain_db;

    if (s_lock)
        xSemaphoreGive(
            s_lock
        );


    return ESP_OK;
}


/* ============================================================
 * Rebuild active chain
 * ============================================================ */

static esp_err_t rebuild_active_chain(void)
{
    dsp_preset_t preset;

    if (s_lock)
        xSemaphoreTake(
            s_lock,
            portMAX_DELAY
        );

    preset =
        s_presets[s_active];

    if (s_lock)
        xSemaphoreGive(
            s_lock
        );

    return apply_preset_as_active(
        &preset
    );
}


/* ============================================================
 * NVS - Preset
 * ============================================================ */

static esp_err_t load_preset_from_nvs(
    nvs_handle_t nvs,
    dsp_preset_id_t id)
{
    char key[8];

    snprintf(
        key,
        sizeof(key),
        "p%d",
        (int)id
    );

    dsp_preset_blob_t blob;

    size_t len = sizeof(blob);

    esp_err_t err =
        nvs_get_blob(
            nvs,
            key,
            &blob,
            &len
        );

    /* A genuinely missing record gets a factory seed. */
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_presets[id] = s_factory_presets[id];
        blob.magic = DSP_PRESET_MAGIC;
        blob.version = DSP_PRESET_VERSION;
        blob.preset = s_factory_presets[id];
        return nvs_set_blob(nvs, key, &blob, sizeof(blob));
    }

    if (err != ESP_OK) {
        s_presets[id] = s_factory_presets[id];
        ESP_LOGW(TAG, "Preset %d NVS read failed (%s); using factory in RAM",
                 (int)id, esp_err_to_name(err));
        return ESP_OK;
    }

    /*
     * Preserve unknown/incompatible blobs instead of overwriting a user's
     * saved data. Factory values are used in RAM until an explicit Save.
     */
    if (len != sizeof(blob) ||
        blob.magic != DSP_PRESET_MAGIC ||
        blob.version != DSP_PRESET_VERSION ||
        !dsp_preset_data_valid(&blob.preset)) {
        s_presets[id] = s_factory_presets[id];
        ESP_LOGW(TAG,
                 "Preset %d NVS blob incompatible/invalid; preserved, factory used in RAM",
                 (int)id);
        return ESP_OK;
    }

    s_presets[id] = blob.preset;
    return ESP_OK;
}


/* ============================================================
 * Init
 * ============================================================ */

esp_err_t dsp_engine_init(void)
{
    if (s_lock)
        return ESP_OK;

    /* Allocate the maximum RAOP PCM frame before playback; never realloc in DSP. */
    if (s_scratch_len < DSP_SCRATCH_MAX_BYTES) {
        uint8_t *new_scratch = malloc(DSP_SCRATCH_MAX_BYTES);
        if (!new_scratch)
            return ESP_ERR_NO_MEM;

        free(s_scratch);
        s_scratch = new_scratch;
        s_scratch_len = DSP_SCRATCH_MAX_BYTES;
    }

    s_lock =
        xSemaphoreCreateMutex();

    if (!s_lock)
        return ESP_ERR_NO_MEM;


    atomic_store_explicit(&s_stream_reset_requested, false, memory_order_relaxed);

    /*
     * Factory defaults.
     *
     * Always start from factory values.
     * NVS will override them below when
     * the saved preset version is valid.
     */

    for (int i = 0;
         i < DSP_NUM_PRESETS;
         i++) {

        s_presets[i] =
            s_factory_presets[i];
    }


    set_character_factory();


    /*
     * Reset PEQ states.
     */

#if CONFIG_DSP_PEQ_USE_ESP_DSP || CONFIG_DSP_PEQ_USE_ESP_DSP_AES3
    memset(s_peq_state, 0, sizeof(s_peq_state));
#else
    memset(s_state, 0, sizeof(s_state));
#endif

    /* Independent output protection, not part of the PEQ/Character bypass. */
    init_subsonic_hpf();

    /*
     * NVS.
     */

    nvs_handle_t nvs;

    esp_err_t err =
        nvs_open(
            DSP_NVS_NAMESPACE,
            NVS_READWRITE,
            &nvs
        );


    if (err == ESP_OK) {

        uint8_t enabled =
            1;


        if (nvs_get_u8(
                nvs,
                DSP_NVS_KEY_ENABLED,
                &enabled
            ) == ESP_OK) {

            s_enabled =
                enabled != 0;
        }


        uint8_t active =
            DSP_PRESET_BEO;


        if (nvs_get_u8(
                nvs,
                DSP_NVS_KEY_ACTIVE,
                &active
            ) == ESP_OK) {

            if (active < DSP_NUM_PRESETS) {

                s_active =
                    (dsp_preset_id_t)active;
            }
        }


        /*
         * Presets.
         *
         * load_preset_from_nvs() checks:
         *
         * 1. NVS exists
         * 2. blob size is correct
         * 3. magic is correct
         * 4. preset version matches
         *
         * If any check fails:
         * current factory preset is written
         * to NVS.
         */

        for (int i = 0;
             i < DSP_NUM_PRESETS;
             i++) {

            load_preset_from_nvs(
                nvs,
                (dsp_preset_id_t)i
            );
        }


        load_character_from_nvs(nvs);


        /*
         * Commit any preset factory migration.
         */

        nvs_commit(nvs);


        nvs_close(nvs);
    }
    else {

        ESP_LOGW(
            TAG,
            "NVS open failed: %s, using factory defaults",
            esp_err_to_name(err)
        );
    }


    /*
     * Build PEQ chain.
     */


    err =
        rebuild_active_chain();

    if (err != ESP_OK)
        return err;


    rebuild_character_filters();
    reset_character_states();
    memset(&s_output_state, 0, sizeof(s_output_state));
    s_output_state.bass_gain = 1.0f;
    s_output_state.treble_gain = 1.0f;


    ESP_LOGI(
        TAG,
        "DSP init: enabled=%d active=%d",
        s_enabled,
        (int)s_active
    );
    ESP_LOGI(TAG,
             "Subsonic HPF: %.0f Hz, 4th-order Butterworth (24 dB/oct), always ON",
             DSP_SUBSONIC_HPF_HZ);
#if CONFIG_DSP_PEQ_USE_ESP_DSP_AES3
    ESP_LOGI(TAG, "PEQ backend: ESP-DSP dual-mono block biquad (AES3 eligible)");
#elif CONFIG_DSP_PEQ_USE_ESP_DSP
    ESP_LOGI(TAG, "PEQ backend: ESP-DSP interleaved stereo block biquad");
#else
    ESP_LOGI(TAG, "PEQ backend: native transposed DF-II");
#endif


    ESP_LOGI(
        TAG,
        "Character V5.1 Lite: enabled=%d mode=%d W=%.0f D=%.0f Drive=%.0f stages=2",
        s_character.enabled, (int)s_character.mode,
        s_character.warmth_pct, s_character.detail_pct,
        s_character.drive_pct
    );

    log_character_switch_boot_state(s_enabled, &s_character);


    return ESP_OK;
}

/* ============================================================
 * DSP enabled
 * ============================================================ */

esp_err_t dsp_engine_set_enabled(
    bool enabled)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    bool previous_enabled;
    dsp_character_t character_snapshot;

    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    previous_enabled = s_enabled;
    character_snapshot = s_character;
    s_enabled = enabled;

    xSemaphoreGive(
        s_lock
    );

    log_character_switch_changes(
        "DSP/master",
        previous_enabled, &character_snapshot,
        enabled, &character_snapshot);


    /*
     * Preserve original behavior:
     * enabled state is saved immediately.
     */

    nvs_handle_t nvs;

    esp_err_t err =
        nvs_open(
            DSP_NVS_NAMESPACE,
            NVS_READWRITE,
            &nvs
        );

    if (err != ESP_OK)
        return err;


    err =
        nvs_set_u8(
            nvs,
            DSP_NVS_KEY_ENABLED,
            enabled ? 1 : 0
        );


    if (err == ESP_OK)
        err =
            nvs_commit(nvs);


    nvs_close(nvs);

    return err;
}

bool dsp_engine_get_enabled(void)
{
    bool enabled;


    if (!s_lock)
        return true;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    enabled =
        s_enabled;

    xSemaphoreGive(
        s_lock
    );


    return enabled;
}


/* ============================================================
 * AirPlay volume reference + Dynamic Loudness
 * Actual volume gain is applied after Stereo Spatial in i2s_output.
 * ============================================================ */

esp_err_t dsp_engine_set_output_volume(float volume)
{
    if (!isfinite(volume))
        return ESP_ERR_INVALID_ARG;

    volume = clampf_local(volume, 0.0f, 1.0f);
    const uint32_t q = (uint32_t)(volume * 1000000.0f + 0.5f);
    atomic_store_explicit(&s_output_volume_q, q, memory_order_release);
    return ESP_OK;
}

float dsp_engine_get_output_volume(void)
{
    return (float)atomic_load_explicit(
        &s_output_volume_q, memory_order_acquire) * 0.000001f;
}

esp_err_t dsp_engine_set_dynamic_loudness(bool enabled)
{
    const bool previous = atomic_exchange_explicit(
        &s_dynamic_loudness_enabled, enabled, memory_order_acq_rel);
    if (previous != enabled) {
        ESP_LOGI(TAG, "Dynamic Loudness: %s", enabled ? "ON" : "OFF");
    }
    return ESP_OK;
}

bool dsp_engine_get_dynamic_loudness(void)
{
    return atomic_load_explicit(
        &s_dynamic_loudness_enabled, memory_order_acquire);
}

esp_err_t dsp_engine_set_dynamic_bass_max_db(uint8_t max_db)
{
    if (max_db > 5U)
        return ESP_ERR_INVALID_ARG;

    atomic_store_explicit(&s_dynamic_bass_max_db, max_db, memory_order_release);
    return ESP_OK;
}

uint8_t dsp_engine_get_dynamic_bass_max_db(void)
{
    return (uint8_t)atomic_load_explicit(
        &s_dynamic_bass_max_db, memory_order_acquire);
}

esp_err_t dsp_engine_set_dynamic_loudness_calibration(
    uint8_t reference_spl_db, float reference_gain)
{
    if ((reference_spl_db != 0U &&
         (reference_spl_db < 35U || reference_spl_db > 80U)) ||
        !isfinite(reference_gain) ||
        reference_gain < 0.001f || reference_gain > 2.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t gain_q =
        (uint32_t)(reference_gain * 1000000.0f + 0.5f);
    atomic_store_explicit(
        &s_dynamic_reference_gain_q, gain_q, memory_order_release);
    atomic_store_explicit(
        &s_dynamic_reference_spl_db, reference_spl_db, memory_order_release);
    return ESP_OK;
}

void dsp_engine_get_dynamic_loudness_calibration(
    uint8_t *reference_spl_db, float *reference_gain)
{
    if (reference_spl_db) {
        *reference_spl_db = (uint8_t)atomic_load_explicit(
            &s_dynamic_reference_spl_db, memory_order_acquire);
    }
    if (reference_gain) {
        *reference_gain = (float)atomic_load_explicit(
            &s_dynamic_reference_gain_q, memory_order_acquire) * 0.000001f;
    }
}


/* ============================================================
 * Active preset
 * ============================================================ */

esp_err_t dsp_engine_select_preset(
    dsp_preset_id_t id)
{
    if (id < 0 ||
        id >= DSP_NUM_PRESETS) {

        return ESP_ERR_INVALID_ARG;
    }


    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    s_active =
        id;

    xSemaphoreGive(
        s_lock
    );


    esp_err_t err =
        rebuild_active_chain();

    if (err != ESP_OK)
        return err;


    /*
     * Preserve original behavior:
     * selecting a preset immediately persists it.
     */

    nvs_handle_t nvs;

    err =
        nvs_open(
            DSP_NVS_NAMESPACE,
            NVS_READWRITE,
            &nvs
        );

    if (err != ESP_OK)
        return err;


    err =
        nvs_set_u8(
            nvs,
            DSP_NVS_KEY_ACTIVE,
            (uint8_t)id
        );


    if (err == ESP_OK)
        err =
            nvs_commit(nvs);


    nvs_close(nvs);

    return err;
}


dsp_preset_id_t dsp_engine_get_active_preset(void)
{
    dsp_preset_id_t id;


    if (!s_lock)
        return DSP_PRESET_BEO;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    id =
        s_active;

    xSemaphoreGive(
        s_lock
    );


    return id;
}


/* ============================================================
 * Compatibility wrappers
 * ============================================================ */

esp_err_t dsp_engine_set_active(
    dsp_preset_id_t id)
{
    return dsp_engine_select_preset(id);
}


dsp_preset_id_t dsp_engine_get_active(void)
{
    return dsp_engine_get_active_preset();
}


esp_err_t dsp_engine_save_active(void)
{
    dsp_preset_id_t id =
        dsp_engine_get_active_preset();

    return dsp_engine_save_preset(id);
}


/* ============================================================
 * Set band
 * ============================================================ */

esp_err_t dsp_engine_set_band(
    dsp_preset_id_t id,
    uint8_t band_idx,
    float freq,
    float gain_db,
    float q)
{
    if (id < 0 ||
        id >= DSP_NUM_PRESETS ||
        !isfinite(freq) || !isfinite(gain_db) || !isfinite(q)) {

        return ESP_ERR_INVALID_ARG;
    }


    if (band_idx >= DSP_MAX_BANDS)
        return ESP_ERR_INVALID_ARG;


    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );


    if (band_idx >=
        s_presets[id].num_bands) {

        xSemaphoreGive(
            s_lock
        );

        return ESP_ERR_INVALID_ARG;
    }


    s_presets[id].bands[band_idx].freq =
        clampf_local(
            freq,
            20.0f,
            20000.0f
        );


    s_presets[id].bands[band_idx].gain_db =
        clampf_local(
            gain_db,
            -12.0f,
            12.0f
        );


    s_presets[id].bands[band_idx].q =
        clampf_local(
            q,
            0.1f,
            10.0f
        );


    const bool active =
        (id == s_active);


    xSemaphoreGive(
        s_lock
    );


    /*
     * Rebuild only if editing active preset.
     */

    if (active)
        return rebuild_active_chain();


    return ESP_OK;
}


/* ============================================================
 * Set shelf
 * ============================================================ */

esp_err_t dsp_engine_set_shelf(
    dsp_preset_id_t id,
    float freq,
    float gain_db)
{
    if (id < 0 ||
        id >= DSP_NUM_PRESETS ||
        !isfinite(freq) || !isfinite(gain_db)) {

        return ESP_ERR_INVALID_ARG;
    }


    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );


    s_presets[id].shelf_freq =
        clampf_local(
            freq,
            1000.0f,
            20000.0f
        );


    s_presets[id].shelf_gain_db =
        clampf_local(
            gain_db,
            -12.0f,
            12.0f
        );


    const bool active =
        (id == s_active);


    xSemaphoreGive(
        s_lock
    );


    if (active)
        return rebuild_active_chain();


    return ESP_OK;
}


/* ============================================================
 * Set master
 * ============================================================ */

esp_err_t dsp_engine_set_master(
    dsp_preset_id_t id,
    float gain_db)
{
    if (id < 0 ||
        id >= DSP_NUM_PRESETS ||
        !isfinite(gain_db)) {

        return ESP_ERR_INVALID_ARG;
    }


    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );


    s_presets[id].master_gain_db =
        clampf_local(
            gain_db,
            -24.0f,
            6.0f
        );


    const bool active =
        (id == s_active);


    xSemaphoreGive(
        s_lock
    );


    if (active)
        return rebuild_active_chain();


    return ESP_OK;
}


/* ============================================================
 * Get preset
 * ============================================================ */

esp_err_t dsp_engine_get_preset(
    dsp_preset_id_t id,
    dsp_preset_t *out)
{
    if (!out)
        return ESP_ERR_INVALID_ARG;


    if (id < 0 ||
        id >= DSP_NUM_PRESETS) {

        return ESP_ERR_INVALID_ARG;
    }


    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    *out =
        s_presets[id];

    xSemaphoreGive(
        s_lock
    );


    return ESP_OK;
}


/* ============================================================
 * Preset name
 * ============================================================ */

const char *dsp_engine_preset_name(
    dsp_preset_id_t id)
{
    if (id < 0 ||
        id >= DSP_NUM_PRESETS) {

        return "UNKNOWN";
    }


    return s_presets[id].name;
}


/* ============================================================
 * Save preset
 * ============================================================ */

esp_err_t dsp_engine_save_preset(
    dsp_preset_id_t id)
{
    if (id < 0 ||
        id >= DSP_NUM_PRESETS) {

        return ESP_ERR_INVALID_ARG;
    }


    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    dsp_preset_t preset;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    preset =
        s_presets[id];

    xSemaphoreGive(
        s_lock
    );


    dsp_preset_blob_t blob;

    blob.magic =
        DSP_PRESET_MAGIC;

    blob.version =
        DSP_PRESET_VERSION;

    blob.preset =
        preset;


    char key[8];

    snprintf(
        key,
        sizeof(key),
        "p%d",
        (int)id
    );


    nvs_handle_t nvs;

    esp_err_t err =
        nvs_open(
            DSP_NVS_NAMESPACE,
            NVS_READWRITE,
            &nvs
        );

    if (err != ESP_OK)
        return err;


    err =
        nvs_set_blob(
            nvs,
            key,
            &blob,
            sizeof(blob)
        );


    if (err == ESP_OK)
        err =
            nvs_commit(nvs);


    nvs_close(nvs);

    return err;
}


/* ============================================================
 * Reset preset
 * ============================================================ */

esp_err_t dsp_engine_reset_preset(
    dsp_preset_id_t id)
{
    if (id < 0 ||
        id >= DSP_NUM_PRESETS) {

        return ESP_ERR_INVALID_ARG;
    }


    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );


    s_presets[id] =
        s_factory_presets[id];


    const bool active =
        (id == s_active);


    xSemaphoreGive(
        s_lock
    );


    if (active) {

        esp_err_t err =
            rebuild_active_chain();

        if (err != ESP_OK)
            return err;
    }


    /*
     * Only reset this preset.
     * Analog Character settings are intentionally untouched.
     */

    return dsp_engine_save_preset(id);
}


/* ============================================================
 * Analog Character API / persistence
 * ============================================================ */

esp_err_t dsp_engine_set_character(const dsp_character_t *config)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;
    if (!character_config_valid(config))
        return ESP_ERR_INVALID_ARG;

    dsp_character_t next = *config;
    /* V5 ignores these legacy V4 controls; retain the NVS struct layout. */
    next.softness_pct = 0.0f;
    /* air_pct is the backward-compatible storage slot for HF H2 amount. */
    next.memory_pct = 0.0f;
    next.spectral_pct = 0.0f;

    dsp_character_t previous;
    bool dsp_enabled;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    previous = s_character;
    dsp_enabled = s_enabled;
    s_character = next;
    xSemaphoreGive(s_lock);

    log_character_switch_changes(
        "API/Character",
        dsp_enabled, &previous,
        dsp_enabled, &next);

    return ESP_OK;
}

esp_err_t dsp_engine_get_character(dsp_character_t *out)
{
    if (!out)
        return ESP_ERR_INVALID_ARG;
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_character;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t dsp_engine_set_character_profile(dsp_character_mode_t mode)
{
    if (mode != DSP_CHARACTER_TUBE && mode != DSP_CHARACTER_BRITISH)
        return ESP_ERR_INVALID_ARG;

    dsp_character_t next;
    character_profile_defaults(mode, &next);
    return dsp_engine_set_character(&next);
}

esp_err_t dsp_engine_save_character(void)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;

    dsp_character_blob_t blob;
    memset(&blob, 0, sizeof(blob));

    xSemaphoreTake(s_lock, portMAX_DELAY);
    blob.character = s_character;
    xSemaphoreGive(s_lock);

    blob.magic = DSP_CHARACTER_MAGIC;
    blob.version = DSP_CHARACTER_VERSION;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(DSP_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK)
        return err;

    err = nvs_set_blob(nvs, DSP_NVS_KEY_CHARACTER, &blob, sizeof(blob));
    if (err == ESP_OK)
        err = nvs_commit(nvs);

    nvs_close(nvs);
    return err;
}


/* ============================================================
 * Optional HF refinement controls (persisted per preset by dsp_api)
 * ============================================================ */

esp_err_t dsp_engine_set_character_hf(
    bool a_enabled, bool b_enabled, uint8_t a_strength_pct, uint8_t b_strength_pct)
{
    if (a_strength_pct > 100U || b_strength_pct > 100U)
        return ESP_ERR_INVALID_ARG;

    const bool old_a = atomic_exchange_explicit(
        &s_character_hf_a_enabled, a_enabled, memory_order_acq_rel);
    const bool old_b = atomic_exchange_explicit(
        &s_character_hf_b_enabled, b_enabled, memory_order_acq_rel);
    const uint32_t old_a_strength = atomic_exchange_explicit(
        &s_character_hf_a_strength_pct, a_strength_pct, memory_order_acq_rel);
    const uint32_t old_b_strength = atomic_exchange_explicit(
        &s_character_hf_b_strength_pct, b_strength_pct, memory_order_acq_rel);

    if (old_a != a_enabled || old_a_strength != a_strength_pct) {
        ESP_LOGI(TAG, "Character HF A: %s strength=%u%% (max transient lift 0.8 dB)",
                 a_enabled ? "ON" : "OFF", (unsigned)a_strength_pct);
    }
    if (old_b != b_enabled || old_b_strength != b_strength_pct) {
        ESP_LOGI(TAG, "Character HF B: %s strength=%u%% (max transient lift 0.5 dB)",
                 b_enabled ? "ON" : "OFF", (unsigned)b_strength_pct);
    }
    return ESP_OK;
}

esp_err_t dsp_engine_get_character_hf(
    bool *a_enabled, bool *b_enabled, uint8_t *a_strength_pct, uint8_t *b_strength_pct)
{
    if (!a_enabled || !b_enabled || !a_strength_pct || !b_strength_pct)
        return ESP_ERR_INVALID_ARG;
    *a_enabled = atomic_load_explicit(&s_character_hf_a_enabled, memory_order_acquire);
    *b_enabled = atomic_load_explicit(&s_character_hf_b_enabled, memory_order_acquire);
    *a_strength_pct = (uint8_t)atomic_load_explicit(
        &s_character_hf_a_strength_pct, memory_order_acquire);
    *b_strength_pct = (uint8_t)atomic_load_explicit(
        &s_character_hf_b_strength_pct, memory_order_acquire);
    return ESP_OK;
}


/* ============================================================
 * Per-preset A/B masks; processing-time counters remain runtime-only
 * ============================================================ */

esp_err_t dsp_engine_set_character_diagnostics(uint8_t stage_mask,
                                               uint8_t feature_mask)
{
    if ((stage_mask & (uint8_t)~DSP_CHARACTER_STAGE_MASK_ALL) != 0 ||
        (feature_mask & (uint8_t)~DSP_CHARACTER_FEATURE_MASK_ALL) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    atomic_store_explicit(
        &s_character_diag_stage_mask, stage_mask, memory_order_release);
    atomic_store_explicit(
        &s_character_diag_feature_mask, feature_mask, memory_order_release);

    /* Start a fresh measurement window after every A/B change. */
    atomic_store_explicit(&s_character_diag_blocks, 0, memory_order_relaxed);
    for (unsigned stage = 0; stage < DSP_CHARACTER_DIAG_SLOTS; ++stage) {
        atomic_store_explicit(
            &s_character_diag_stage_total_us[stage], 0, memory_order_relaxed);
        atomic_store_explicit(
            &s_character_diag_stage_max_us[stage], 0, memory_order_relaxed);
    }

    return ESP_OK;
}

void dsp_engine_request_stream_reset(void)
{
    atomic_store_explicit(
        &s_stream_reset_requested, true, memory_order_release);
}

esp_err_t dsp_engine_get_character_diagnostic_masks(uint8_t *stage_mask,
                                                     uint8_t *feature_mask)
{
    if (!stage_mask || !feature_mask)
        return ESP_ERR_INVALID_ARG;

    *stage_mask = (uint8_t)atomic_load_explicit(
        &s_character_diag_stage_mask, memory_order_acquire);
    *feature_mask = (uint8_t)atomic_load_explicit(
        &s_character_diag_feature_mask, memory_order_acquire);
    return ESP_OK;
}

esp_err_t dsp_engine_get_character_diagnostics(dsp_character_diagnostics_t *out)
{
    if (!out)
        return ESP_ERR_INVALID_ARG;

    memset(out, 0, sizeof(*out));
    out->stage_mask = (uint8_t)atomic_load_explicit(
        &s_character_diag_stage_mask, memory_order_acquire);
    out->feature_mask = (uint8_t)atomic_load_explicit(
        &s_character_diag_feature_mask, memory_order_acquire);
    out->blocks = atomic_exchange_explicit(
        &s_character_diag_blocks, 0, memory_order_acq_rel);
    out->cpu_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;

    for (unsigned stage = 0; stage < DSP_CHARACTER_DIAG_SLOTS; ++stage) {
        const uint32_t total_us = atomic_exchange_explicit(
            &s_character_diag_stage_total_us[stage], 0, memory_order_acq_rel);
        out->stage_avg_us[stage] = out->blocks ? total_us / out->blocks : 0;
        out->stage_max_us[stage] = atomic_exchange_explicit(
            &s_character_diag_stage_max_us[stage], 0, memory_order_acq_rel);
    }

    return ESP_OK;
}


/* ============================================================
 * Audio processing
 * ============================================================ */

static const uint8_t * IRAM_ATTR dsp_engine_process_internal(
    const uint8_t *pcm_in,
    uint8_t *pcm_out,
    size_t len_bytes,
    size_t *out_len)
{
    if (!out_len)
        return pcm_in;


    *out_len =
        len_bytes;


    if (!pcm_in ||
        len_bytes < 4 ||
        (len_bytes & 0x03) != 0) {

        return pcm_in;
    }

    if (atomic_exchange_explicit(
            &s_stream_reset_requested, false, memory_order_acq_rel)) {
        reset_stream_processing_state();
    }

    /* Local DSP configuration copied from the cached snapshot. */
    bool enabled;
    bool character_enabled;
    uint8_t num_stages;
    float master_linear;
    biquad_coef_t coefs[DSP_NUM_STAGES];
    dsp_character_t character;
    character_filter_coef_t character_filters;

    /*
     * Non-blocking configuration snapshot.
     *
     * The audio task is the only caller of dsp_engine_process(). If a web/API
     * task briefly owns s_lock, reuse the last complete snapshot instead of
     * stalling the real-time consumer and backing up the RAOP ring.
     */
    static struct {
        bool valid;
        bool enabled;
        bool character_enabled;
        uint8_t num_stages;
        float master_linear;
        float low_peak_eq_gain_db;
        float high_peak_eq_gain_db;
        biquad_coef_t coefs[DSP_NUM_STAGES];
        dsp_character_t character;
        character_filter_coef_t character_filters;
    } cached;

    if (!s_lock)
        return pcm_in;

    if (xSemaphoreTake(s_lock, 0) == pdTRUE) {
        cached.enabled = s_enabled;
        cached.num_stages = s_active_num_stages;
        cached.master_linear = s_active_master_linear;
        cached.low_peak_eq_gain_db = s_active_low_peak_eq_gain_db;
        cached.high_peak_eq_gain_db = s_active_high_peak_eq_gain_db;
        memcpy(cached.coefs, s_active_coefs, sizeof(cached.coefs));

        cached.character = s_character;
        cached.character_enabled = cached.character.enabled;
        cached.character_filters = s_character_filters;
        cached.valid = true;

        xSemaphoreGive(s_lock);
    } else if (!cached.valid) {
        /* Startup contention: pass this first frame through rather than wait. */
        return pcm_in;
    }

    enabled = cached.enabled;
    num_stages = cached.num_stages;
    master_linear = cached.master_linear;
    memcpy(coefs, cached.coefs, sizeof(coefs));
    character = cached.character;
    character_enabled = cached.character_enabled;
    character_filters = cached.character_filters;

    /* HF controls are atomically snapshotted once per PCM block. */
    const bool character_hf_a_requested = atomic_load_explicit(
        &s_character_hf_a_enabled, memory_order_acquire);
    const bool character_hf_b_requested = atomic_load_explicit(
        &s_character_hf_b_enabled, memory_order_acquire);
    const float character_hf_a_strength = (float)atomic_load_explicit(
        &s_character_hf_a_strength_pct, memory_order_acquire) * 0.01f;
    const float character_hf_b_strength = (float)atomic_load_explicit(
        &s_character_hf_b_strength_pct, memory_order_acquire) * 0.01f;
    const bool dynamic_loudness_requested = atomic_load_explicit(
        &s_dynamic_loudness_enabled, memory_order_acquire);
    /* Avoid even the volume atomic read when this optional feature is OFF. */
    const float target_output_volume = dynamic_loudness_requested
        ? (float)atomic_load_explicit(
            &s_output_volume_q, memory_order_acquire) * 0.000001f
        : 1.0f;

    /* Estimate current 1 kHz phon from the measured calibration point. */
    float target_loudness_bass = 1.0f;
    float target_loudness_treble = 1.0f;
    if (dynamic_loudness_requested && target_output_volume > 0.0001f) {
        const float current_effective_gain = fmaxf(
            target_output_volume * master_linear, 0.001f);
        const float reference_gain = fmaxf(
            (float)atomic_load_explicit(
                &s_dynamic_reference_gain_q, memory_order_acquire) * 0.000001f,
            0.001f);
        const uint32_t stored_spl = atomic_load_explicit(
            &s_dynamic_reference_spl_db, memory_order_acquire);
        const float reference_phon = (stored_spl >= 35U && stored_spl <= 90U)
            ? (float)stored_spl : 60.0f;
        const float current_phon = clampf_local(
            reference_phon +
                20.0f * log10f(current_effective_gain / reference_gain),
            20.0f, 80.0f);
        const float bass_max_db = (float)atomic_load_explicit(
            &s_dynamic_bass_max_db, memory_order_acquire);

        /* ISO 226:2023 contour at 110 Hz (log interpolation around 100–125 Hz). */
        const float bass_iso_delta_db = iso226_relative_compensation_db(
            current_phon, reference_phon,
            0.42403038f, -7.38846f, 24.62065f);
        /* Bass is boost-only: it returns to the original response as volume rises. */
        const float bass_compensation_db = clampf_local(
            bass_iso_delta_db, 0.0f, bass_max_db);

        /* ISO 226:2023 contour around 11 kHz (interpolated 10–12.5 kHz). */
        const float treble_compensation_db = clampf_local(
            iso226_relative_compensation_db(
                current_phon, reference_phon,
                0.3362370f, -7.73975f, 13.2166f),
            -2.0f, 1.5f);

        target_loudness_bass = db_to_linear(bass_compensation_db);
        target_loudness_treble = db_to_linear(treble_compensation_db);
    }


    /*
     * Global bypass.
     */

    if (!enabled) {
        if (s_character_state.running)
            reset_character_states();
        else if (s_character_hf_state.history_active)
            memset(&s_character_hf_state, 0, sizeof(s_character_hf_state));

        /*
         * Keep subsonic protection, output master trim and optional
         * low-volume compensation active in global bypass. AirPlay volume
         * is applied after Stereo Spatial by the final I2S output stage.
         */
        int16_t *bypass_out = (int16_t *)(pcm_out ? pcm_out : s_scratch);
        if (!pcm_out) {
            if (!s_scratch || len_bytes > s_scratch_len)
                return pcm_in;
            memcpy(bypass_out, pcm_in, len_bytes);
        }

        const size_t bypass_frames = len_bytes / 4U;
        const bool loudness_active =
            fabsf(target_loudness_bass - 1.0f) > 0.0001f ||
            fabsf(target_loudness_treble - 1.0f) > 0.0001f ||
            fabsf(s_output_state.bass_gain - 1.0f) > 0.0001f ||
            fabsf(s_output_state.treble_gain - 1.0f) > 0.0001f;

        for (size_t frame = 0; frame < bypass_frames; ++frame) {
            if (loudness_active) {
                s_output_state.bass_gain += character_filters.loudness_gain_alpha *
                    (target_loudness_bass - s_output_state.bass_gain);
                s_output_state.treble_gain += character_filters.loudness_gain_alpha *
                    (target_loudness_treble - s_output_state.treble_gain);
            }

            float left = (float)bypass_out[frame * 2U] * (1.0f / 32768.0f);
            float right = (float)bypass_out[frame * 2U + 1U] * (1.0f / 32768.0f);

            left = subsonic_hpf_process_sample(left, 0U);
            right = subsonic_hpf_process_sample(right, 1U);

            if (loudness_active) {
                if (!s_output_state.filter_initialized[0]) {
                    s_output_state.low_lp[0] = left;
                    s_output_state.high_lp[0] = left;
                    s_output_state.filter_initialized[0] = true;
                }
                if (!s_output_state.filter_initialized[1]) {
                    s_output_state.low_lp[1] = right;
                    s_output_state.high_lp[1] = right;
                    s_output_state.filter_initialized[1] = true;
                }

                s_output_state.low_lp[0] += character_filters.loudness_low_alpha *
                    (left - s_output_state.low_lp[0]);
                s_output_state.low_lp[1] += character_filters.loudness_low_alpha *
                    (right - s_output_state.low_lp[1]);
                s_output_state.high_lp[0] += character_filters.loudness_high_alpha *
                    (left - s_output_state.high_lp[0]);
                s_output_state.high_lp[1] += character_filters.loudness_high_alpha *
                    (right - s_output_state.high_lp[1]);

                left += s_output_state.low_lp[0] * (s_output_state.bass_gain - 1.0f) +
                    (left - s_output_state.high_lp[0]) * (s_output_state.treble_gain - 1.0f);
                right += s_output_state.low_lp[1] * (s_output_state.bass_gain - 1.0f) +
                    (right - s_output_state.high_lp[1]) * (s_output_state.treble_gain - 1.0f);
            }

            /* Bypass omits PEQ/Character but retains smoothed preset master. */
            s_output_state.master_gain = smooth_master_gain(
                s_output_state.master_gain, master_linear);
            left *= s_output_state.master_gain;
            right *= s_output_state.master_gain;

            /* Stereo-linked soft-knee limiter: one gain preserves L/R balance. */
            const float knee = 0.94f;
            const float range = 0.045f;
            const float peak = fmaxf(fabsf(left), fabsf(right));
            if (peak > knee) {
                const float over = peak - knee;
                const float limited_peak =
                    knee + over / (1.0f + over / range);
                const float linked_gain = limited_peak / peak;
                left *= linked_gain;
                right *= linked_gain;
            }

            left = clampf_local(left, -0.985f, 0.985f);
            right = clampf_local(right, -0.985f, 0.985f);
            bypass_out[frame * 2U] = float_to_pcm16_rounded(left);
            bypass_out[frame * 2U + 1U] = float_to_pcm16_rounded(right);
        }

        if (fabsf(target_loudness_bass - 1.0f) < 0.0001f &&
            fabsf(target_loudness_treble - 1.0f) < 0.0001f &&
            fabsf(s_output_state.bass_gain - 1.0f) < 0.0002f &&
            fabsf(s_output_state.treble_gain - 1.0f) < 0.0002f) {
            s_output_state.bass_gain = 1.0f;
            s_output_state.treble_gain = 1.0f;
            s_output_state.filter_initialized[0] = false;
            s_output_state.filter_initialized[1] = false;
        }

        return pcm_out ? pcm_out : s_scratch;
    }


    const uint32_t diag_stage_mask =
        atomic_load_explicit(&s_character_diag_stage_mask, memory_order_relaxed) &
        DSP_CHARACTER_STAGE_MASK_ALL;
    const uint32_t diag_feature_mask =
        atomic_load_explicit(&s_character_diag_feature_mask, memory_order_relaxed) &
        DSP_CHARACTER_FEATURE_MASK_ALL;

    const bool diag_warmth = (diag_feature_mask & DSP_CHARACTER_FEATURE_WARMTH) != 0;
    const bool diag_detail = (diag_feature_mask & DSP_CHARACTER_FEATURE_DETAIL) != 0;
    const bool diag_drive = (diag_feature_mask & DSP_CHARACTER_FEATURE_DRIVE) != 0;

    const size_t frames =
        len_bytes / 4;


    const size_t required =
        len_bytes;


    /*
     * Scratch buffer.
     */

    if (!pcm_out && (!s_scratch || required > s_scratch_len)) {
        ESP_LOGE(
            TAG,
            "PCM frame exceeds preallocated DSP scratch: %u > %u",
            (unsigned)required,
            (unsigned)s_scratch_len
        );
        return pcm_in;
    }


#if !CONFIG_DSP_PEQ_USE_ESP_DSP && !CONFIG_DSP_PEQ_USE_ESP_DSP_AES3
    const int16_t *in =
        (const int16_t *)pcm_in;
#endif

    /*
     * Phase 2: the pipeline tap may supply its DSP-ring frame as the
     * destination. In-place operation removes the scratch-to-frame copy.
     * The legacy API still uses s_scratch and keeps its original semantics.
     */
    int16_t *out =
        (int16_t *)(pcm_out ? pcm_out : s_scratch);

#if CONFIG_DSP_PEQ_USE_ESP_DSP_AES3
    if (frames > DSP_PEQ_MAX_FRAMES) {
        return pcm_in;
    }

    /* PCM16 -> planar float channels. */
    const int16_t *pcm_samples = (const int16_t *)pcm_in;
    const float pcm_scale = 1.0f / 32768.0f;
    for (size_t frame = 0; frame < frames; ++frame) {
        s_peq_left[frame] = (float)pcm_samples[frame * 2U] * pcm_scale;
        s_peq_right[frame] = (float)pcm_samples[frame * 2U + 1U] * pcm_scale;
    }

    /*
     * Run each channel through the mono DF-II block kernel. On current
     * ESP-DSP releases, dsps_biquad_f32 can dispatch to the ESP32-S3 AES3
     * assembly when CONFIG_DSP_OPTIMIZED selects optimized kernels.
     */
    for (uint8_t stage = 0; stage < num_stages; ++stage) {
        float *coeff = (float *)&coefs[stage];
        if (dsps_biquad_f32(s_peq_left, s_peq_left, (int)frames,
                            coeff, s_peq_state[stage][0]) != ESP_OK ||
            dsps_biquad_f32(s_peq_right, s_peq_right, (int)frames,
                            coeff, s_peq_state[stage][1]) != ESP_OK) {
            return pcm_in;
        }
    }
#elif CONFIG_DSP_PEQ_USE_ESP_DSP
    if ((frames * 2U) > DSP_PEQ_MAX_FLOATS) {
        return pcm_in;
    }

    /* PCM16 -> interleaved float once per block. */
    const int16_t *pcm_samples = (const int16_t *)pcm_in;
    const float pcm_scale = 1.0f / 32768.0f;
    for (size_t sample = 0; sample < frames * 2U; ++sample) {
        s_peq_interleaved[sample] = (float)pcm_samples[sample] * pcm_scale;
    }

    /*
     * Run the complete PEQ cascade block-wise. ESP-DSP's stereo kernel
     * consumes L/R interleaved samples and keeps two DF-II states/channel.
     * Filter coefficients are unchanged; only the state realization differs
     * from the native transposed DF-II implementation.
     */
    for (uint8_t stage = 0; stage < num_stages; ++stage) {
        if (dsps_biquad_sf32(
                s_peq_interleaved,
                s_peq_interleaved,
                (int)frames,
                (float *)&coefs[stage],
                s_peq_state[stage]) != ESP_OK) {
            return pcm_in;
        }
    }
#endif


    /*
     * V6 Phase 1: preserve the 12 ms exponential control time constant at
     * the block boundary, then linearly ramp the derived coefficients across
     * this PCM block. The detector, ADAA and nonlinear transfer are unchanged.
     */
    const float character_control_smooth = 0.00188786049f;
    static size_t cached_control_frames = 0;
    static float cached_block_alpha = 0.0f;
    static float cached_inv_frames = 0.0f;
    if (frames != cached_control_frames) {
        const float block_decay = powf(
            1.0f - character_control_smooth, (float)frames);
        cached_block_alpha = 1.0f - block_decay;
        cached_inv_frames = 1.0f / (float)frames;
        cached_control_frames = frames;
    }
    const float block_alpha = cached_block_alpha;
    const float inv_frames = cached_inv_frames;

    const bool loudness_active =
        fabsf(target_loudness_bass - 1.0f) > 0.0001f ||
        fabsf(target_loudness_treble - 1.0f) > 0.0001f ||
        fabsf(s_output_state.bass_gain - 1.0f) > 0.0001f ||
        fabsf(s_output_state.treble_gain - 1.0f) > 0.0001f;

    character_state_t *cs = &s_character_state;

    const float target_warmth =
        (character_enabled && diag_warmth)
        ? character.warmth_pct * 0.01f : 0.0f;
    const float target_detail =
        (character_enabled && diag_detail)
        ? character.detail_pct * 0.01f : 0.0f;
    const float target_drive =
        (character_enabled && diag_drive)
        ? character.drive_pct * 0.01f : 0.0f;
    const float target_interaction =
        (character_enabled && character.harmonic_interaction_enabled)
        ? character.harmonic_interaction_pct * 0.01f : 0.0f;
    const float target_dynamic =
        (character_enabled && character.dynamic_saturation_enabled)
        ? character.dynamic_saturation_pct * 0.01f : 0.0f;
    const float target_air_harmonic =
        (character_enabled && diag_detail)
        ? character.air_pct * 0.01f : 0.0f;

    const float warmth_start = cs->warmth;
    const float detail_start = cs->detail;
    const float drive_start = cs->drive;
    const float interaction_start = cs->interaction;
    const float dynamic_start = cs->dynamic_saturation;
    const float air_start = cs->air_harmonic;

    const float warmth_end =
        warmth_start + block_alpha * (target_warmth - warmth_start);
    const float detail_end =
        detail_start + block_alpha * (target_detail - detail_start);
    const float drive_end =
        drive_start + block_alpha * (target_drive - drive_start);
    const float interaction_end =
        interaction_start + block_alpha * (target_interaction - interaction_start);
    const float dynamic_end =
        dynamic_start + block_alpha * (target_dynamic - dynamic_start);
    const float air_end =
        air_start + block_alpha * (target_air_harmonic - air_start);

    cs->warmth = warmth_end;
    cs->detail = detail_end;
    cs->drive = drive_end;
    cs->interaction = interaction_end;
    cs->dynamic_saturation = dynamic_end;
    cs->air_harmonic = air_end;

    float current_air_harmonic = air_start;
    const float air_harmonic_step = (air_end - air_start) * inv_frames;

    character_gain_coeffs_t gain_start;
    character_gain_coeffs_t gain_end;
    character_calculate_gain_coeffs(
        warmth_start, detail_start, drive_start,
        interaction_start, dynamic_start, diag_drive, &gain_start);
    character_calculate_gain_coeffs(
        warmth_end, detail_end, drive_end,
        interaction_end, dynamic_end, diag_drive, &gain_end);

    float current_h2_gain[CHARACTER_STAGE_COUNT] = {
        gain_start.h2_gain[0], gain_start.h2_gain[1]
    };
    float current_h3_gain[CHARACTER_STAGE_COUNT] = {
        gain_start.h3_gain[0], gain_start.h3_gain[1]
    };
    const float h2_gain_step[CHARACTER_STAGE_COUNT] = {
        (gain_end.h2_gain[0] - gain_start.h2_gain[0]) * inv_frames,
        (gain_end.h2_gain[1] - gain_start.h2_gain[1]) * inv_frames
    };
    const float h3_gain_step[CHARACTER_STAGE_COUNT] = {
        (gain_end.h3_gain[0] - gain_start.h3_gain[0]) * inv_frames,
        (gain_end.h3_gain[1] - gain_start.h3_gain[1]) * inv_frames
    };

    float current_im_guard = gain_start.im_guard_amount;
    const float im_guard_step =
        (gain_end.im_guard_amount - gain_start.im_guard_amount) * inv_frames;
    float current_dynamic_scale = gain_start.dynamic_scale;
    const float dynamic_scale_step =
        (gain_end.dynamic_scale - gain_start.dynamic_scale) * inv_frames;
    float current_interaction_scale = gain_start.interaction_scale;
    const float interaction_scale_step =
        (gain_end.interaction_scale - gain_start.interaction_scale) * inv_frames;

    const float max_dynamic_scale = fmaxf(
        gain_start.dynamic_scale, gain_end.dynamic_scale);
    const float max_interaction_scale = fmaxf(
        gain_start.interaction_scale, gain_end.interaction_scale);
    const bool dynamic_active = max_dynamic_scale > 0.0000002f;

    uint8_t active_stage_mask = 0;
    for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
        const float max_gain = fmaxf(
            gain_start.h2_gain[stage] + gain_start.h3_gain[stage],
            gain_end.h2_gain[stage] + gain_end.h3_gain[stage]);
        if ((diag_stage_mask & (1U << stage)) &&
            max_gain > 0.0000001f) {
            active_stage_mask |= (uint8_t)(1U << stage);
        }
    }

    const bool interaction_for_stage2 =
        (active_stage_mask & DSP_CHARACTER_STAGE_1) != 0 &&
        max_interaction_scale > 0.00000025f;
    const uint8_t stages_just_disabled =
        s_character_active_stage_mask & (uint8_t)(~active_stage_mask);

    /* Prevent stale bypassed-stage residuals entering the shared HPF. */
    if (active_stage_mask != s_character_active_stage_mask) {
        memset(s_character_residual_hpf, 0,
               sizeof(s_character_residual_hpf));
    }

    for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
        if (stages_just_disabled & (1U << stage)) {
            memset(&cs->stage[stage], 0, sizeof(cs->stage[stage]));
        }
    }
    s_character_active_stage_mask = active_stage_mask;

#if CONFIG_DSP_CHARACTER_PROFILE
    /* Opt-in only: cycle-counter reads stay out of normal playback builds. */
    uint32_t character_stage_cycles[DSP_CHARACTER_DIAG_SLOTS] = {0};
#endif

    for (size_t i = 0;
         i < frames;
         i++) {
        current_h2_gain[0] += h2_gain_step[0];
        current_h2_gain[1] += h2_gain_step[1];
        current_h3_gain[1] += h3_gain_step[1];
        current_air_harmonic += air_harmonic_step;
        current_im_guard += im_guard_step;
        if (dynamic_active)
            current_dynamic_scale += dynamic_scale_step;
        if (interaction_for_stage2)
            current_interaction_scale += interaction_scale_step;

        if (loudness_active) {
            s_output_state.bass_gain += character_filters.loudness_gain_alpha *
                (target_loudness_bass - s_output_state.bass_gain);
            s_output_state.treble_gain += character_filters.loudness_gain_alpha *
                (target_loudness_treble - s_output_state.treble_gain);
        }

#if CONFIG_DSP_PEQ_USE_ESP_DSP_AES3
        float yl = s_peq_left[i];
        float yr = s_peq_right[i];
#elif CONFIG_DSP_PEQ_USE_ESP_DSP
        float yl = s_peq_interleaved[i * 2U];
        float yr = s_peq_interleaved[i * 2U + 1U];
#else
        float yl = (float)in[i * 2U] * (1.0f / 32768.0f);
        float yr = (float)in[i * 2U + 1U] * (1.0f / 32768.0f);

        /* Native transposed DF-II PEQ cascade (reference backend). */
        for (uint8_t stage = 0; stage < num_stages; ++stage) {
            const biquad_coef_t *c = &coefs[stage];
            yl = biquad_process_sample(yl, c, &s_state[stage][0]);
            yr = biquad_process_sample(yr, c, &s_state[stage][1]);
        }
#endif


        /*
         * ----------------------------------------------------
         * Analog Character V5.1 Lite — two-stage parallel model
         * ----------------------------------------------------
         * Stage 1: 80–450 Hz, H2 body.
         * Stage 2: 250 Hz–1.4 kHz, H2 + controllable H3.
         *
         * Harmonic Interaction coordinates Stage 2 against Stage 1 energy.
         * Dynamic Saturation adds a bounded level-dependent harmonic mix.
         * Both controls are optional; no extra nonlinear/filter stage is added.
         */
        {
            character_state_t *cs = &s_character_state;
            const bool character_mix_active =
                (active_stage_mask != 0) || cs->running;

            if (character_mix_active) {
                const float dry[2] = { yl, yr };
                float residual_sum[2] = { 0.0f, 0.0f };

                /* Very gentle 0.3 dB supply-sag analogue, applied to generated harmonics only. */
                const float linked_input_peak = fmaxf(fabsf(dry[0]), fabsf(dry[1]));
                const float sag_env_alpha =
                    (linked_input_peak > cs->sag_envelope)
                    ? character_filters.sag_attack_alpha
                    : character_filters.sag_release_alpha;
                cs->sag_envelope += sag_env_alpha *
                    (linked_input_peak - cs->sag_envelope);
                const float sag_depth_db = 0.30f * clampf_local(
                    (cs->sag_envelope - 0.40f) * (1.0f / 0.55f), 0.0f, 1.0f);
                /* Second-order exp approximation over a maximum 0.3 dB range;
                 * avoids powf() in the per-sample real-time loop. */
                const float sag_x = 0.11512925f * sag_depth_db;
                const float sag_target_gain =
                    1.0f - sag_x + 0.5f * sag_x * sag_x;
                const float sag_gain_alpha =
                    (sag_target_gain < cs->sag_gain)
                    ? character_filters.sag_attack_alpha
                    : character_filters.sag_release_alpha;
                cs->sag_gain += sag_gain_alpha * (sag_target_gain - cs->sag_gain);

                for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
                    if (!(active_stage_mask & (1U << stage)))
                        continue;

#if CONFIG_DSP_CHARACTER_PROFILE
                    const uint32_t stage_start_cycles = esp_cpu_get_cycle_count();
#endif
                    character_stage_state_t *ss = &cs->stage[stage];
                    const character_stage_filter_t *fc =
                        &character_filters.stage[stage];

                    const bool interaction_for_stage =
                        (stage == 1) && interaction_for_stage2;

                    for (unsigned ch = 0; ch < 2; ++ch) {
                        float band = biquad_process_sample(
                            dry[ch], &fc->input_hpf, &ss->input_hpf[ch]);
                        band = biquad_process_sample(
                            band, &fc->input_lpf, &ss->input_lpf[ch]);

                        const float band_power = band * band;
                        ss->mean_square[ch] += fc->ms_alpha *
                            (band_power - ss->mean_square[ch]);
                        /* The power follower is a convex average of nonnegative
                         * samples, so it cannot become negative in normal operation. */
                        const float mean_square = ss->mean_square[ch];

                        float even_gain = current_h2_gain[stage];
                        float odd_gain = current_h3_gain[stage];

                        const float level_raw = mean_square * 32.0f;
                        const float level_norm = (level_raw < 1.0f)
                            ? level_raw : 1.0f;
                        const float im_guard = 1.0f -
                            current_im_guard * level_norm;

                        /* IM guard remains active in every path; optional
                         * modifiers and their clamp are skipped when OFF. */
                        float harmonic_scale = im_guard;
                        if (dynamic_active) {
                            harmonic_scale *=
                                1.0f + current_dynamic_scale * level_norm;
                        }
                        if (interaction_for_stage) {
                            const float lower_ms = cs->stage[0].mean_square[ch];
                            const float lower_raw = lower_ms * 48.0f;
                            const float lower_norm = (lower_raw < 1.0f)
                                ? lower_raw : 1.0f;
                            harmonic_scale *=
                                1.0f - current_interaction_scale * lower_norm;
                        }
                        if (dynamic_active || interaction_for_stage) {
                            harmonic_scale = clampf_local(
                                harmonic_scale, 0.65f, 1.25f);
                        }
                        even_gain *= harmonic_scale;
                        odd_gain *= harmonic_scale;

                        float residual;
                        if (ss->adaa_valid[ch]) {
                            const float aa_even_gain =
                                0.5f * (even_gain + ss->prev_even_gain[ch]);
                            const float aa_odd_gain =
                                0.5f * (odd_gain + ss->prev_odd_gain[ch]);
                            if (stage == 0 || aa_odd_gain == 0.0f) {
                                residual = character_adaa_even_residual(
                                    band, band_power, ss->prev_x[ch],
                                    ss->prev_band_power[ch],
                                    mean_square, ss->prev_mean_square[ch],
                                    aa_even_gain);
                            } else {
                                residual = character_adaa_poly_residual(
                                    band, band_power, ss->prev_x[ch],
                                    ss->prev_band_power[ch],
                                    mean_square, ss->prev_mean_square[ch],
                                    aa_even_gain, aa_odd_gain);
                            }
                        } else {
                            residual = 0.0f;
                            if (even_gain != 0.0f) {
                                residual += even_gain *
                                    (band_power - mean_square);
                            }
                            if (odd_gain != 0.0f) {
                                residual += odd_gain *
                                    (band_power * band - 1.5f * mean_square * band);
                            }
                            ss->adaa_valid[ch] = true;
                        }

                        ss->prev_x[ch] = band;
                        ss->prev_band_power[ch] = band_power;
                        ss->prev_even_gain[ch] = even_gain;
                        ss->prev_odd_gain[ch] = odd_gain;
                        ss->prev_mean_square[ch] = mean_square;

                        residual_sum[ch] += residual;
                    }

#if CONFIG_DSP_CHARACTER_PROFILE
                    character_stage_cycles[stage] +=
                        (uint32_t)(esp_cpu_get_cycle_count() - stage_start_cycles);
#endif
                }

                /* Both stages use identical 15 Hz HPFs. By linearity,
                 * H(r1) + H(r2) == H(r1 + r2), so filter the sum once. */
#if CONFIG_DSP_CHARACTER_PROFILE
                const uint32_t residual_filter_start = esp_cpu_get_cycle_count();
#endif
                for (unsigned ch = 0; ch < 2; ++ch) {
                    residual_sum[ch] = biquad_process_sample(
                        residual_sum[ch], &character_filters.residual_hpf,
                        &s_character_residual_hpf[ch]);
                }
#if CONFIG_DSP_CHARACTER_PROFILE
                const uint32_t residual_filter_cycles =
                    (uint32_t)(esp_cpu_get_cycle_count() - residual_filter_start);
                if (active_stage_mask & DSP_CHARACTER_STAGE_1) {
                    character_stage_cycles[0] += residual_filter_cycles;
                } else if (active_stage_mask & DSP_CHARACTER_STAGE_2) {
                    character_stage_cycles[1] += residual_filter_cycles;
                }
#endif

                yl = dry[0] + residual_sum[0] * cs->sag_gain;
                yr = dry[1] + residual_sum[1] * cs->sag_gain;

                cs->running = (active_stage_mask != 0);
                if (!cs->running) {
                    memset(cs->stage, 0, sizeof(cs->stage));
                    memset(s_character_residual_hpf, 0,
                           sizeof(s_character_residual_hpf));
                    /* Clear the sag envelope so a later re-enable starts at unity. */
                    cs->sag_envelope = 0.0f;
                    cs->sag_gain = 1.0f;
                    s_character_active_stage_mask = 0;
                }
            }
        }

        /*
         * ----------------------------------------------------
         * Analog Character HF refinement — transient-only A/B
         * ----------------------------------------------------
         * A: 2–5 kHz articulation, maximum +0.8 dB on detected attacks.
         * B: 5–10 kHz micro-transients, maximum +0.5 dB on detected attacks.
         * Fast/slow magnitude envelopes suppress steady-state hiss; no
         * cubic harmonics, static EQ lift, allocation, or audio-task locks.
         */
        {
            character_hf_state_t *hf = &s_character_hf_state;
            const float target_a = character_hf_a_requested ? character_hf_a_strength : 0.0f;
            const float target_b = character_hf_b_requested ? character_hf_b_strength : 0.0f;
            hf->mix_a += character_filters.hf_mix_alpha * (target_a - hf->mix_a);
            hf->mix_b += character_filters.hf_mix_alpha * (target_b - hf->mix_b);

            const bool a_active =
                target_a > 0.00001f || hf->mix_a > 0.00001f;
            const bool b_active =
                target_b > 0.00001f || hf->mix_b > 0.00001f;
            const bool air_active = current_air_harmonic > 0.00001f;
            const bool hf_active = a_active || b_active || air_active;

#if CONFIG_DSP_CHARACTER_PROFILE
            const uint32_t hf_stage_start_cycles =
                hf_active ? esp_cpu_get_cycle_count() : 0U;
#endif

            if (hf_active) {
                hf->history_active = true;
                for (unsigned ch = 0; ch < 2; ++ch) {
                    const float x = (ch == 0) ? yl : yr;

                    /* A detector is also the low-band reference for B's guard. */
                    const float band_a = biquad_process_sample(
                        x, &character_filters.hf_band_a, &hf->band_a[ch]);
                    const float abs_a = fabsf(band_a);

                    float transient_a = 0.0f;
                    if (a_active) {
                        float alpha = (abs_a > hf->fast_env_a[ch])
                            ? character_filters.hf_fast_attack_alpha
                            : character_filters.hf_fast_release_alpha;
                        hf->fast_env_a[ch] += alpha *
                            (abs_a - hf->fast_env_a[ch]);
                    } else {
                        /* B-only mode needs the slow A reference, not A attack tracking. */
                        hf->fast_env_a[ch] = 0.0f;
                    }

                    float alpha = (abs_a > hf->slow_env_a[ch])
                        ? character_filters.hf_slow_attack_alpha
                        : character_filters.hf_slow_release_alpha;
                    hf->slow_env_a[ch] += alpha *
                        (abs_a - hf->slow_env_a[ch]);

                    if (a_active) {
                        transient_a = clampf_local(
                            (hf->fast_env_a[ch] -
                             1.25f * hf->slow_env_a[ch] - 0.0006f) /
                            (hf->fast_env_a[ch] + 0.0012f), 0.0f, 1.0f);
                    }

                    float y = x;
                    if (a_active)
                        y += band_a * transient_a * hf->mix_a * 0.09648f;

                    if (air_active) {
                        const float band_a_power = band_a * band_a;
                        hf->air_mean_square[ch] += character_filters.hf_air_alpha *
                            (band_a_power - hf->air_mean_square[ch]);
                        const float h2_residual = biquad_process_sample(
                            band_a_power - hf->air_mean_square[ch],
                            &character_filters.hf_air_hpf, &hf->air_hpf[ch]);
                        const float air_gate = clampf_local(
                            (hf->slow_env_a[ch] - 0.0015f) * 125.0f,
                            0.0f, 1.0f);
                        /* A tiny H2 residual from 2–5 kHz lands mainly in 4–10 kHz. */
                        y += h2_residual * current_air_harmonic * 0.18f * air_gate;
                    } else {
                        memset(&hf->air_hpf[ch], 0, sizeof(hf->air_hpf[ch]));
                        hf->air_mean_square[ch] = 0.0f;
                    }

                    if (b_active) {
                        const float band_b = biquad_process_sample(
                            x, &character_filters.hf_band_b, &hf->band_b[ch]);
                        const float abs_b = fabsf(band_b);

                        alpha = (abs_b > hf->fast_env_b[ch])
                            ? character_filters.hf_fast_attack_alpha
                            : character_filters.hf_fast_release_alpha;
                        hf->fast_env_b[ch] += alpha *
                            (abs_b - hf->fast_env_b[ch]);
                        alpha = (abs_b > hf->slow_env_b[ch])
                            ? character_filters.hf_slow_attack_alpha
                            : character_filters.hf_slow_release_alpha;
                        hf->slow_env_b[ch] += alpha *
                            (abs_b - hf->slow_env_b[ch]);

                        const float transient_b = clampf_local(
                            (hf->fast_env_b[ch] -
                             1.30f * hf->slow_env_b[ch] - 0.0009f) /
                            (hf->fast_env_b[ch] + 0.0015f), 0.0f, 1.0f);

                        /* Partial guard only: cymbals can share this spectrum. */
                        const float hf_sib_ratio = hf->slow_env_b[ch] /
                            (hf->slow_env_a[ch] + 0.0005f);
                        const float hf_sib_guard = 1.0f - 0.35f *
                            clampf_local(
                                (hf_sib_ratio - 0.75f) / 1.25f, 0.0f, 1.0f);

                        y += band_b * transient_b * hf->mix_b *
                            0.05925f * hf_sib_guard;
                    } else {
                        /* Prevent stale B history from reappearing when enabled later. */
                        memset(&hf->band_b[ch], 0, sizeof(hf->band_b[ch]));
                        hf->fast_env_b[ch] = 0.0f;
                        hf->slow_env_b[ch] = 0.0f;
                    }

                    if (ch == 0) yl = y;
                    else yr = y;
                }
            } else if (hf->history_active) {
                memset(hf, 0, sizeof(*hf));
            }

#if CONFIG_DSP_CHARACTER_PROFILE
            if (hf_active) {
                character_stage_cycles[2] +=
                    (uint32_t)(esp_cpu_get_cycle_count() - hf_stage_start_cycles);
            }
#endif
        }

        /*
         * ----------------------------------------------------
         * Fixed subsonic protection
         * ----------------------------------------------------
         * Applied after PEQ + Character and before master/limiter.
         * Both channels use identical coefficients and independent states.
         */
        yl = subsonic_hpf_process_sample(yl, 0U);
        yr = subsonic_hpf_process_sample(yr, 1U);

        if (loudness_active) {
            if (!s_output_state.filter_initialized[0]) {
                s_output_state.low_lp[0] = yl;
                s_output_state.high_lp[0] = yl;
                s_output_state.filter_initialized[0] = true;
            }
            if (!s_output_state.filter_initialized[1]) {
                s_output_state.low_lp[1] = yr;
                s_output_state.high_lp[1] = yr;
                s_output_state.filter_initialized[1] = true;
            }

            s_output_state.low_lp[0] += character_filters.loudness_low_alpha *
                (yl - s_output_state.low_lp[0]);
            s_output_state.low_lp[1] += character_filters.loudness_low_alpha *
                (yr - s_output_state.low_lp[1]);
            s_output_state.high_lp[0] += character_filters.loudness_high_alpha *
                (yl - s_output_state.high_lp[0]);
            s_output_state.high_lp[1] += character_filters.loudness_high_alpha *
                (yr - s_output_state.high_lp[1]);

            yl += s_output_state.low_lp[0] * (s_output_state.bass_gain - 1.0f) +
                (yl - s_output_state.high_lp[0]) * (s_output_state.treble_gain - 1.0f);
            yr += s_output_state.low_lp[1] * (s_output_state.bass_gain - 1.0f) +
                (yr - s_output_state.high_lp[1]) * (s_output_state.treble_gain - 1.0f);
        } else {
            s_output_state.filter_initialized[0] = false;
            s_output_state.filter_initialized[1] = false;
        }

        /*
         * ----------------------------------------------------
         * Preset master trim. AirPlay volume is applied after Stereo Spatial
         * by the final I2S output stage.
         * ----------------------------------------------------
         */
        s_output_state.master_gain = smooth_master_gain(
            s_output_state.master_gain, master_linear);
        yl *= s_output_state.master_gain;
        yr *= s_output_state.master_gain;


        /*
         * Soft-knee peak control. Linear below 0.94, asymptotically
         * approaches 0.985 above it; avoids the old hard-clipping edge.
         */
        /* Stereo-linked soft-knee limiter: one gain preserves L/R balance. */
        const float limiter_knee = 0.94f;
        const float limiter_range = 0.045f;
        const float stereo_peak = fmaxf(fabsf(yl), fabsf(yr));
        if (stereo_peak > limiter_knee) {
            const float over = stereo_peak - limiter_knee;
            const float limited_peak =
                limiter_knee + over / (1.0f + over / limiter_range);
            const float linked_gain = limited_peak / stereo_peak;
            yl *= linked_gain;
            yr *= linked_gain;
        }

        yl = clampf_local(yl, -0.985f, 0.985f);
        yr = clampf_local(yr, -0.985f, 0.985f);


        out[i * 2] = float_to_pcm16_rounded(yl);
        out[i * 2 + 1] = float_to_pcm16_rounded(yr);
    }


    if (fabsf(target_loudness_bass - 1.0f) < 0.0001f &&
        fabsf(target_loudness_treble - 1.0f) < 0.0001f &&
        fabsf(s_output_state.bass_gain - 1.0f) < 0.0002f &&
        fabsf(s_output_state.treble_gain - 1.0f) < 0.0002f) {
        s_output_state.bass_gain = 1.0f;
        s_output_state.treble_gain = 1.0f;
        s_output_state.filter_initialized[0] = false;
        s_output_state.filter_initialized[1] = false;
    }

#if CONFIG_DSP_CHARACTER_PROFILE
    if (character_enabled || character_hf_a_requested || character_hf_b_requested) {
        const uint32_t cpu_mhz =
            (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ > 0)
            ? CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ : 240U;

        for (unsigned stage = 0; stage < DSP_CHARACTER_DIAG_SLOTS; ++stage) {
            const uint32_t elapsed_us = character_stage_cycles[stage] / cpu_mhz;
            atomic_fetch_add_explicit(
                &s_character_diag_stage_total_us[stage],
                elapsed_us,
                memory_order_relaxed);

            uint32_t previous_max = atomic_load_explicit(
                &s_character_diag_stage_max_us[stage],
                memory_order_relaxed);
            while (elapsed_us > previous_max &&
                   !atomic_compare_exchange_weak_explicit(
                       &s_character_diag_stage_max_us[stage],
                       &previous_max,
                       elapsed_us,
                       memory_order_relaxed,
                       memory_order_relaxed)) {
            }
        }
        atomic_fetch_add_explicit(
            &s_character_diag_blocks, 1U, memory_order_relaxed);
    }
#endif

    return pcm_out ? pcm_out : s_scratch;
}

/* Compatibility API: retain the original non-mutating scratch-buffer path. */
const uint8_t *dsp_engine_process(
    const uint8_t *pcm_in,
    size_t len_bytes,
    size_t *out_len)
{
    return dsp_engine_process_internal(
        pcm_in, NULL, len_bytes, out_len);
}

/* Real-time pipeline API: process the writable DSP-ring frame in-place. */
void dsp_engine_process_inplace(
    uint8_t *pcm,
    size_t len_bytes)
{
    size_t out_len = len_bytes;
    (void)dsp_engine_process_internal(
        pcm, pcm, len_bytes, &out_len);
}