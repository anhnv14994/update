#include "dsp_engine.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdatomic.h>

#include "esp_cpu.h"
#include "sdkconfig.h"

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

#define DSP_NVS_NAMESPACE    "dsp_cfg"
#define DSP_NVS_KEY_ENABLED  "enabled"
#define DSP_NVS_KEY_ACTIVE   "active"
#define DSP_SCRATCH_MAX_BYTES 2048U

#define DSP_PRESET_MAGIC     0x44535031u   /* "DSP1" */
#define DSP_PRESET_VERSION   2u


/* ============================================================
 * Exciter
 * ============================================================ */

#define EXC_H2_MIN              0.00f
#define EXC_H2_MAX              0.12f

#define EXC_H3_MIN              0.00f
#define EXC_H3_MAX              0.06f

#define EXC_CROSSOVER_MIN       30.0f
#define EXC_CROSSOVER_MAX       100.0f

/*
 * Harmonic band limiting.
 *
 * The HPF removes the low-frequency fundamental/DC component
 * which can otherwise leak into the nonlinear branch.
 */
#define EXC_HARMONIC_HPF_HZ     65.0f
#define EXC_HARMONIC_HPF_MAX_HZ  120.0f
#define EXC_H2_LPF_HZ           160.0f
#define EXC_H3_LPF_HZ           180.0f


/*
 * Envelope follower.
 *
 * Used only as a gentle activity detector.  The nonlinear
 * generator is no longer fully normalized to this envelope;
 * this prevents low-level passages from being turned into a
 * nearly constant synthetic bass layer.
 */
#define EXC_ENV_ATTACK_MS       5.0f
#define EXC_ENV_RELEASE_MS      100.0f
#define EXC_ENV_FLOOR           0.015f
#define EXC_ENV_GATE             0.025f
#define EXC_ENV_SOFT_START       0.080f
#define EXC_NONLINEAR_DRIVE      1.75f


/* Exciter NVS */

#define DSP_EXCITER_MAGIC    0x45584331u   /* "EXC1" */
#define DSP_NVS_KEY_EXCITER  "exciter"


#define DSP_CHARACTER_MAGIC   0x43485231u   /* "CHR1" */
#define DSP_CHARACTER_VERSION 2u
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
    uint32_t       magic;
    dsp_exciter_t  exciter;
} dsp_exciter_blob_t;


typedef struct {
    uint32_t          magic;
    uint32_t          version;
    dsp_character_t   character;
} dsp_character_blob_t;

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

static biquad_state_t s_state[DSP_NUM_STAGES][2];


/* ============================================================
 * Scratch buffer
 * ============================================================ */

static uint8_t *s_scratch = NULL;
static size_t   s_scratch_len = 0;


/* ============================================================
 * Exciter runtime
 * ============================================================ */

static dsp_exciter_t s_exciter = {
    .enabled      = true,
    .h2_pct       = 0.06f,
    .h3_pct       = 0.02f,
    .crossover_hz = 60.0f,
};


/*
 * Exciter filter chain:
 *
 * bass
 *   -> crossover LPF
 *
 * harmonic branch:
 *
 * H2 -> HPF 65 -> LPF 160
 * H3 -> HPF 65 -> LPF 180
 */
typedef struct {

    biquad_coef_t bass_lpf;

    biquad_coef_t h2_hpf;
    biquad_coef_t h2_lpf;

    biquad_coef_t h3_hpf;
    biquad_coef_t h3_lpf;

} exciter_filter_coef_t;


typedef struct {

    biquad_state_t bass_lpf;

    biquad_state_t h2_hpf;
    biquad_state_t h2_lpf;

    biquad_state_t h3_hpf;
    biquad_state_t h3_lpf;

    /*
     * Slow peak envelope of the low-frequency bass.
     */
    float envelope;

} exciter_state_t;


static exciter_filter_coef_t s_exciter_filter;

static exciter_state_t s_exciter_state = {
    .envelope = 0.0f,
};


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

#define CHARACTER_STAGE_COUNT 3

typedef struct {
    biquad_coef_t input_hpf;
    biquad_coef_t input_lpf;
    biquad_coef_t residual_hpf; /* remove nonlinear residual DC */
    biquad_coef_t residual_lpf;
    float env_attack_alpha;
    float env_release_alpha;
    float charge_attack_alpha;
    float charge_release_alpha;
    float bias_attack_alpha;
    float bias_release_alpha;
    float coupling_alpha;
} character_stage_filter_t;

typedef struct {
    character_stage_filter_t stage[CHARACTER_STAGE_COUNT];
    float air_alpha;
    float supply_attack_alpha;
    float supply_release_alpha;
} character_filter_coef_t;

typedef struct {
    biquad_state_t input_hpf[2];
    biquad_state_t input_lpf[2];
    biquad_state_t residual_hpf[2];
    biquad_state_t residual_lpf[2];
    float envelope[2];
    float charge[2];
    float coupling[2];
    float bias[2];
    float prev_x[2];
    float prev_even_gain[2];
    float prev_odd_gain[2];
    float prev_peak[2];
    bool adaa_valid[2];
} character_stage_state_t;

typedef struct {
    character_stage_state_t stage[CHARACTER_STAGE_COUNT];
    float supply_envelope; /* stereo-linked slow shared supply memory */
    float warmth;
    float detail;
    float drive;
    float softness;
    float memory;
    float spectral;
    float air_gain;
    float air_lp[2];
    bool running;
} character_state_t;


static character_filter_coef_t s_character_filters;
static character_state_t s_character_state;

/* Runtime-only diagnostic switches; deliberately excluded from NVS. */
static _Atomic uint32_t s_character_diag_stage_mask = DSP_CHARACTER_STAGE_MASK_ALL;
static _Atomic uint32_t s_character_diag_feature_mask = DSP_CHARACTER_FEATURE_MASK_ALL;
static _Atomic uint32_t s_character_diag_blocks = 0;
static _Atomic uint32_t s_character_diag_stage_total_us[CHARACTER_STAGE_COUNT] = {0};
static _Atomic uint32_t s_character_diag_stage_max_us[CHARACTER_STAGE_COUNT] = {0};

/* Audio-task-owned state used to clear stale histories after a test bypass. */
static uint8_t s_character_active_stage_mask = 0;
static bool s_character_memory_feature_active = true;


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


static inline float db_to_linear(float db)
{
    return powf(10.0f, db / 20.0f);
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
 * Exciter filter rebuild
 * ============================================================ */

static void rebuild_exciter_filters(void)
{
    dsp_exciter_t exciter;
    exciter_filter_coef_t next;

    if (!s_lock)
        return;

    /* Snapshot settings first; all coefficient math is lock-free. */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    exciter = s_exciter;
    xSemaphoreGive(s_lock);

    /*
     * Bass crossover: clean 2nd-order Butterworth.
     */
    calc_butterworth_lpf_q(
        DSP_SAMPLE_RATE_HZ,
        exciter.crossover_hz,
        0.70710678118f,
        &next.bass_lpf
    );

    /*
     * The harmonic branch must start above the actual bass
     * fundamental region.  Keep a 65 Hz floor, but move the HPF
     * upward with crossover so XO=80..100 Hz cannot re-introduce
     * the fundamental as synthetic bass.
     */
    float harmonic_hpf =
        fmaxf(
            EXC_HARMONIC_HPF_HZ,
            exciter.crossover_hz * 1.10f
        );

    harmonic_hpf =
        clampf_local(
            harmonic_hpf,
            EXC_HARMONIC_HPF_HZ,
            EXC_HARMONIC_HPF_MAX_HZ
        );

    /*
     * H2: fundamental rejection + focused second-harmonic band.
     * H3: fundamental rejection + slightly wider upper harmonic band.
     * Q=0.707 keeps the sub character tight and avoids resonant bass.
     */
    calc_butterworth_hpf_q(
        DSP_SAMPLE_RATE_HZ,
        harmonic_hpf,
        0.70710678118f,
        &next.h2_hpf
    );

    calc_butterworth_lpf_q(
        DSP_SAMPLE_RATE_HZ,
        EXC_H2_LPF_HZ,
        0.70710678118f,
        &next.h2_lpf
    );

    calc_butterworth_hpf_q(
        DSP_SAMPLE_RATE_HZ,
        harmonic_hpf,
        0.70710678118f,
        &next.h3_hpf
    );

    calc_butterworth_lpf_q(
        DSP_SAMPLE_RATE_HZ,
        EXC_H3_LPF_HZ,
        0.70710678118f,
        &next.h3_lpf
    );

    /*
     * Atomic coefficient publication.  The audio task snapshots
     * the complete structure under the same mutex, so it can never
     * see half of one filter set and half of another.
     */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_exciter_filter = next;
    xSemaphoreGive(s_lock);
}

/* ============================================================
 * Reset Exciter states
 * ============================================================ */

static void reset_exciter_states(void)
{
    memset(
        &s_exciter_state,
        0,
        sizeof(s_exciter_state)
    );
}


/* ============================================================
 * Analog Character V4.1 — low-cost polynomial ADAA
 * ============================================================
 * Each stage uses a band-limited source and an asymmetric quadratic/cubic
 * transfer. First-order ADAA is evaluated in closed form: no LUT access,
 * logarithm, or per-sample division. Slow bias/coupling/supply states provide
 * memory; no audio-rate sample feedback is used.
 *
 * For f(x)=a2*x^2+a3*x^3, the exact first-order ADAA interval averages are:
 *   AA2 = (x^2 + x*p + p^2) / 3
 *   AA3 = (x^3 + x^2*p + x*p^2 + p^3) / 4
 */
static inline float character_adaa_poly_residual(
    float x, float x_prev,
    float peak, float peak_prev,
    float even_gain, float odd_gain)
{
    const float p = x_prev;
    const float x2 = x * x;
    const float p2 = p * p;
    const float xp = x * p;

    const float aa2 = (x2 + xp + p2) * 0.333333333333f;

    const float aa3 = (x2 * x + x2 * p + x * p2 + p2 * p) * 0.25f;
    const float mid = (x + p) * 0.5f;

    /* Remove the steady-sine DC/fundamental components. The residual HPF
     * also rejects DC and slow envelope error for complex programme material. */
    const float peak_avg = (peak + peak_prev) * 0.5f;
    const float peak2 = peak_avg * peak_avg;
    const float h2 = aa2 - 0.5f * peak2;
    const float h3 = aa3 - 0.75f * peak2 * mid;

    return even_gain * h2 + odd_gain * h3;
}

static void rebuild_character_filters(void)
{
    character_filter_coef_t next;
    memset(&next, 0, sizeof(next));

    /* Stage 1: low-mid body. Stage 2: vocal core. Stage 3: presence texture.
     * The residual paths remain parallel to each stage's dry signal. */
    static const float source_hp[CHARACTER_STAGE_COUNT] = { 90.0f, 320.0f, 1700.0f };
    static const float source_lp[CHARACTER_STAGE_COUNT] = { 650.0f, 2100.0f, 4600.0f };
    static const float residual_lp[CHARACTER_STAGE_COUNT] = { 2600.0f, 7600.0f, 10500.0f };
    static const float env_attack_ms[CHARACTER_STAGE_COUNT] = { 12.0f, 7.0f, 4.0f };
    static const float env_release_ms[CHARACTER_STAGE_COUNT] = { 140.0f, 100.0f, 70.0f };
    static const float charge_attack_ms[CHARACTER_STAGE_COUNT] = { 55.0f, 45.0f, 35.0f };
    static const float charge_release_ms[CHARACTER_STAGE_COUNT] = { 520.0f, 420.0f, 320.0f };
    static const float bias_attack_ms[CHARACTER_STAGE_COUNT] = { 12.0f, 9.0f, 6.0f };
    static const float bias_release_ms[CHARACTER_STAGE_COUNT] = { 180.0f, 140.0f, 100.0f };
    static const float coupling_hz[CHARACTER_STAGE_COUNT] = { 18.0f, 22.0f, 28.0f };

    for (unsigned s = 0; s < CHARACTER_STAGE_COUNT; ++s) {
        calc_butterworth_hpf_q(DSP_SAMPLE_RATE_HZ, source_hp[s],
                               0.70710678118f, &next.stage[s].input_hpf);
        calc_butterworth_lpf_q(DSP_SAMPLE_RATE_HZ, source_lp[s],
                               0.70710678118f, &next.stage[s].input_lpf);
        calc_butterworth_hpf_q(DSP_SAMPLE_RATE_HZ, 15.0f,
                               0.70710678118f, &next.stage[s].residual_hpf);
        calc_butterworth_lpf_q(DSP_SAMPLE_RATE_HZ, residual_lp[s],
                               0.70710678118f, &next.stage[s].residual_lpf);
        next.stage[s].env_attack_alpha = 1.0f - expf(
            -1.0f / (env_attack_ms[s] * 0.001f * DSP_SAMPLE_RATE_HZ));
        next.stage[s].env_release_alpha = 1.0f - expf(
            -1.0f / (env_release_ms[s] * 0.001f * DSP_SAMPLE_RATE_HZ));
        next.stage[s].charge_attack_alpha = 1.0f - expf(
            -1.0f / (charge_attack_ms[s] * 0.001f * DSP_SAMPLE_RATE_HZ));
        next.stage[s].charge_release_alpha = 1.0f - expf(
            -1.0f / (charge_release_ms[s] * 0.001f * DSP_SAMPLE_RATE_HZ));
        next.stage[s].bias_attack_alpha = 1.0f - expf(
            -1.0f / (bias_attack_ms[s] * 0.001f * DSP_SAMPLE_RATE_HZ));
        next.stage[s].bias_release_alpha = 1.0f - expf(
            -1.0f / (bias_release_ms[s] * 0.001f * DSP_SAMPLE_RATE_HZ));
        next.stage[s].coupling_alpha = 1.0f - expf(
            -2.0f * (float)M_PI * coupling_hz[s] / DSP_SAMPLE_RATE_HZ);
    }

    next.air_alpha = 1.0f - expf(
        -2.0f * (float)M_PI * 7800.0f / DSP_SAMPLE_RATE_HZ);
    next.supply_attack_alpha = 1.0f - expf(
        -1.0f / (0.250f * DSP_SAMPLE_RATE_HZ));
    next.supply_release_alpha = 1.0f - expf(
        -1.0f / (1.200f * DSP_SAMPLE_RATE_HZ));

    if (s_lock)
        xSemaphoreTake(s_lock, portMAX_DELAY);
    s_character_filters = next;
    if (s_lock)
        xSemaphoreGive(s_lock);
}

static void reset_character_states(void)
{
    memset(&s_character_state, 0, sizeof(s_character_state));
    s_character_state.drive = 0.0f;
    s_character_active_stage_mask = 0;
    s_character_memory_feature_active = false;
}

static void set_character_factory(void)
{
    memset(&s_character, 0, sizeof(s_character));
    s_character.mode = DSP_CHARACTER_CUSTOM;
    s_character.memory_pct = 45.0f;
    s_character.spectral_pct = 50.0f;
}

static bool character_config_valid(const dsp_character_t *c)
{
    if (!c || c->mode < DSP_CHARACTER_CUSTOM ||
        c->mode > DSP_CHARACTER_BRITISH)
        return false;
    if (!isfinite(c->warmth_pct) || !isfinite(c->detail_pct) ||
        !isfinite(c->drive_pct) || !isfinite(c->softness_pct) ||
        !isfinite(c->air_pct) || !isfinite(c->memory_pct) ||
        !isfinite(c->spectral_pct))
        return false;
    return c->warmth_pct >= 0.0f && c->warmth_pct <= 100.0f &&
           c->detail_pct >= 0.0f && c->detail_pct <= 100.0f &&
           c->drive_pct >= 0.0f && c->drive_pct <= 100.0f &&
           c->softness_pct >= 0.0f && c->softness_pct <= 100.0f &&
           c->air_pct >= 0.0f && c->air_pct <= 100.0f &&
           c->memory_pct >= 0.0f && c->memory_pct <= 100.0f &&
           c->spectral_pct >= 0.0f && c->spectral_pct <= 100.0f;
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
        out->softness_pct = 55.0f;
        out->air_pct = 22.0f;
        out->memory_pct = 64.0f;
        out->spectral_pct = 42.0f;
    } else {
        out->warmth_pct = 34.0f;
        out->detail_pct = 18.0f;
        out->drive_pct = 18.0f;
        out->softness_pct = 28.0f;
        out->air_pct = 38.0f;
        out->memory_pct = 30.0f;
        out->spectral_pct = 58.0f;
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
            return ESP_OK;
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
            s_character.softness_pct = old_blob.character.softness_pct;
            s_character.air_pct = old_blob.character.air_pct;
            s_character.memory_pct = 45.0f;
            s_character.spectral_pct = 50.0f;
            if (character_config_valid(&s_character)) {
                dsp_character_blob_t migrated = {
                    .magic = DSP_CHARACTER_MAGIC,
                    .version = DSP_CHARACTER_VERSION,
                    .character = s_character
                };
                (void)nvs_set_blob(nvs, DSP_NVS_KEY_CHARACTER,
                                   &migrated, sizeof(migrated));
                ESP_LOGI(TAG, "Character V3 NVS migrated to V4.1 defaults");
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

static esp_err_t apply_preset_as_active(
    const dsp_preset_t *preset)
{
    if (!preset)
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

    /*
     * Không có preset / blob cũ / version cũ:
     * dùng factory preset mới.
     */
    if (err != ESP_OK ||
        len != sizeof(blob) ||
        blob.magic != DSP_PRESET_MAGIC ||
        blob.version != DSP_PRESET_VERSION) {

        s_presets[id] =
            s_factory_presets[id];

        /*
         * Ghi đè ngay factory preset mới vào NVS.
         */
        blob.magic =
            DSP_PRESET_MAGIC;

        blob.version =
            DSP_PRESET_VERSION;

        blob.preset =
            s_factory_presets[id];

        err =
            nvs_set_blob(
                nvs,
                key,
                &blob,
                sizeof(blob)
            );

        return err;
    }

    /*
     * Version hiện tại:
     * giữ nguyên preset user đã Save.
     */
    s_presets[id] =
        blob.preset;

    return ESP_OK;
}


/* ============================================================
 * NVS - Exciter
 * ============================================================ */

static void set_exciter_factory(void)
{
    s_exciter.enabled =
        true;

    s_exciter.h2_pct =
        0.06f;

    s_exciter.h3_pct =
        0.02f;

    s_exciter.crossover_hz =
        60.0f;
}


static esp_err_t load_exciter_from_nvs(
    nvs_handle_t nvs)
{
    dsp_exciter_blob_t blob;

    size_t len =
        sizeof(blob);


    esp_err_t err =
        nvs_get_blob(
            nvs,
            DSP_NVS_KEY_EXCITER,
            &blob,
            &len
        );


    if (err != ESP_OK ||
        len != sizeof(blob) ||
        blob.magic != DSP_EXCITER_MAGIC) {

        set_exciter_factory();

        return ESP_OK;
    }


    s_exciter =
        blob.exciter;


    s_exciter.h2_pct =
        clampf_local(
            s_exciter.h2_pct,
            EXC_H2_MIN,
            EXC_H2_MAX
        );


    s_exciter.h3_pct =
        clampf_local(
            s_exciter.h3_pct,
            EXC_H3_MIN,
            EXC_H3_MAX
        );


    s_exciter.crossover_hz =
        clampf_local(
            s_exciter.crossover_hz,
            EXC_CROSSOVER_MIN,
            EXC_CROSSOVER_MAX
        );


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


    set_exciter_factory();
    set_character_factory();


    /*
     * Reset PEQ states.
     */

    memset(
        s_state,
        0,
        sizeof(s_state)
    );


    /*
     * Reset Exciter states.
     */

    reset_exciter_states();


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


        /*
         * Exciter.
         */

        load_exciter_from_nvs(
            nvs
        );

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


    /*
     * Build Exciter filters.
     */

    rebuild_exciter_filters();
    reset_exciter_states();

    rebuild_character_filters();
    reset_character_states();


    ESP_LOGI(
        TAG,
        "DSP init: enabled=%d active=%d",
        s_enabled,
        (int)s_active
    );


    ESP_LOGI(
        TAG,
        "Exciter: enabled=%d H2=%.1f%% H3=%.1f%% XO=%.1fHz",
        s_exciter.enabled,
        s_exciter.h2_pct * 100.0f,
        s_exciter.h3_pct * 100.0f,
        s_exciter.crossover_hz
    );


    ESP_LOGI(
        TAG,
        "Character V4.1: enabled=%d mode=%d W=%.0f D=%.0f Drive=%.0f Soft=%.0f Air=%.0f Mem=%.0f Spectral=%.0f",
        s_character.enabled, (int)s_character.mode,
        s_character.warmth_pct, s_character.detail_pct,
        s_character.drive_pct, s_character.softness_pct,
        s_character.air_pct, s_character.memory_pct,
        s_character.spectral_pct
    );


    ESP_LOGI(
        TAG,
        "Exciter filters: dynamic HPF=%.1f..%.1fHz H2_LPF=%.1fHz H3_LPF=%.1fHz",
        EXC_HARMONIC_HPF_HZ,
        EXC_HARMONIC_HPF_MAX_HZ,
        EXC_H2_LPF_HZ,
        EXC_H3_LPF_HZ
    );


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


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    s_enabled =
        enabled;

    xSemaphoreGive(
        s_lock
    );


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
        id >= DSP_NUM_PRESETS) {

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
        id >= DSP_NUM_PRESETS) {

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
        id >= DSP_NUM_PRESETS) {

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
     * Exciter is intentionally untouched.
     */

    return dsp_engine_save_preset(id);
}


/* ============================================================
 * Exciter setters
 * ============================================================ */

esp_err_t dsp_engine_set_exciter(
    bool enabled,
    float h2_pct,
    float h3_pct,
    float crossover_hz)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    h2_pct =
        clampf_local(
            h2_pct,
            EXC_H2_MIN,
            EXC_H2_MAX
        );


    h3_pct =
        clampf_local(
            h3_pct,
            EXC_H3_MIN,
            EXC_H3_MAX
        );


    crossover_hz =
        clampf_local(
            crossover_hz,
            EXC_CROSSOVER_MIN,
            EXC_CROSSOVER_MAX
        );


    bool crossover_changed;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );


    crossover_changed =
        fabsf(
            s_exciter.crossover_hz -
            crossover_hz
        ) > 0.001f;


    s_exciter.enabled =
        enabled;

    s_exciter.h2_pct =
        h2_pct;

    s_exciter.h3_pct =
        h3_pct;

    s_exciter.crossover_hz =
        crossover_hz;


    xSemaphoreGive(
        s_lock
    );


    /*
     * Crossover changes require rebuilding
     * the bass LPF.
     */
    if (crossover_changed) {

        rebuild_exciter_filters();

        reset_exciter_states();
    }


    /*
     * IMPORTANT:
     *
     * Realtime UI changes are NOT saved here.
     *
     * NVS is written only by
     * dsp_engine_save_exciter().
     */

    return ESP_OK;
}


esp_err_t dsp_engine_get_exciter(
    dsp_exciter_t *out)
{
    if (!out)
        return ESP_ERR_INVALID_ARG;


    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    *out =
        s_exciter;

    xSemaphoreGive(
        s_lock
    );


    return ESP_OK;
}


esp_err_t dsp_engine_set_exciter_enabled(
    bool enabled)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    s_exciter.enabled =
        enabled;

    xSemaphoreGive(
        s_lock
    );


    return ESP_OK;
}


esp_err_t dsp_engine_set_exciter_h2(
    float h2_pct)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    h2_pct =
        clampf_local(
            h2_pct,
            EXC_H2_MIN,
            EXC_H2_MAX
        );


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    s_exciter.h2_pct =
        h2_pct;

    xSemaphoreGive(
        s_lock
    );


    return ESP_OK;
}


esp_err_t dsp_engine_set_exciter_h3(
    float h3_pct)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    h3_pct =
        clampf_local(
            h3_pct,
            EXC_H3_MIN,
            EXC_H3_MAX
        );


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    s_exciter.h3_pct =
        h3_pct;

    xSemaphoreGive(
        s_lock
    );


    return ESP_OK;
}


esp_err_t dsp_engine_set_exciter_crossover(
    float crossover_hz)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    crossover_hz =
        clampf_local(
            crossover_hz,
            EXC_CROSSOVER_MIN,
            EXC_CROSSOVER_MAX
        );


    bool changed;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );


    changed =
        fabsf(
            s_exciter.crossover_hz -
            crossover_hz
        ) > 0.001f;


    s_exciter.crossover_hz =
        crossover_hz;


    xSemaphoreGive(
        s_lock
    );


    if (changed) {

        rebuild_exciter_filters();

        reset_exciter_states();
    }


    return ESP_OK;
}


/* ============================================================
 * Save Exciter
 * ============================================================ */

esp_err_t dsp_engine_save_exciter(void)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;


    dsp_exciter_t exciter;


    xSemaphoreTake(
        s_lock,
        portMAX_DELAY
    );

    exciter =
        s_exciter;

    xSemaphoreGive(
        s_lock
    );


    dsp_exciter_blob_t blob;

    blob.magic =
        DSP_EXCITER_MAGIC;

    blob.exciter =
        exciter;


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
            DSP_NVS_KEY_EXCITER,
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
 * Analog Character API / persistence
 * ============================================================ */

esp_err_t dsp_engine_set_character(const dsp_character_t *config)
{
    if (!s_lock)
        return ESP_ERR_INVALID_STATE;
    if (!character_config_valid(config))
        return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_character = *config;
    xSemaphoreGive(s_lock);

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
 * Character diagnostics (runtime only; never persisted)
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
    for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
        atomic_store_explicit(
            &s_character_diag_stage_total_us[stage], 0, memory_order_relaxed);
        atomic_store_explicit(
            &s_character_diag_stage_max_us[stage], 0, memory_order_relaxed);
    }

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

    for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
        const uint32_t total_us = atomic_exchange_explicit(
            &s_character_diag_stage_total_us[stage], 0, memory_order_acq_rel);
        out->stage_avg_us[stage] = out->blocks ? total_us / out->blocks : 0;
        out->stage_max_us[stage] = atomic_exchange_explicit(
            &s_character_diag_stage_max_us[stage], 0, memory_order_acq_rel);
    }

    return ESP_OK;
}


/* ============================================================
 * Exciter envelope coefficients
 * ============================================================ */

static inline float exciter_attack_alpha(void)
{
    return 1.0f -
           expf(
               -1.0f /
               (
                   (EXC_ENV_ATTACK_MS * 0.001f)
                   * DSP_SAMPLE_RATE_HZ
               )
           );
}


static inline float exciter_release_alpha(void)
{
    return 1.0f -
           expf(
               -1.0f /
               (
                   (EXC_ENV_RELEASE_MS * 0.001f)
                   * DSP_SAMPLE_RATE_HZ
               )
           );
}


/* ============================================================
 * Audio processing
 * ============================================================ */

const uint8_t *dsp_engine_process(
    const uint8_t *pcm_in,
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


    /* Local DSP configuration copied from the cached snapshot. */
    bool enabled;
    bool exc_enabled;
    bool character_enabled;
    uint8_t num_stages;
    float master_linear;

    biquad_coef_t coefs[DSP_NUM_STAGES];
    dsp_exciter_t exciter;
    dsp_character_t character;
    exciter_filter_coef_t exc_filters;
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
        bool exc_enabled;
        bool character_enabled;
        uint8_t num_stages;
        float master_linear;
        biquad_coef_t coefs[DSP_NUM_STAGES];
        dsp_exciter_t exciter;
        dsp_character_t character;
        exciter_filter_coef_t exc_filters;
        character_filter_coef_t character_filters;
    } cached;

    if (!s_lock)
        return pcm_in;

    if (xSemaphoreTake(s_lock, 0) == pdTRUE) {
        cached.enabled = s_enabled;
        cached.num_stages = s_active_num_stages;
        cached.master_linear = s_active_master_linear;
        memcpy(cached.coefs, s_active_coefs, sizeof(cached.coefs));

        cached.exciter = s_exciter;
        cached.exc_enabled = cached.exciter.enabled;
        cached.character = s_character;
        cached.character_enabled = cached.character.enabled;
        cached.exc_filters = s_exciter_filter;
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
    exciter = cached.exciter;
    exc_enabled = cached.exc_enabled;
    character = cached.character;
    character_enabled = cached.character_enabled;
    exc_filters = cached.exc_filters;
    character_filters = cached.character_filters;


    /*
     * Global bypass.
     */

    if (!enabled) {
        /* Do not retain a nonlinear state across an arbitrary-length bypass. */
        reset_character_states();
        return pcm_in;
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
    const bool diag_softness = (diag_feature_mask & DSP_CHARACTER_FEATURE_SOFTNESS) != 0;
    const bool diag_air = (diag_feature_mask & DSP_CHARACTER_FEATURE_AIR) != 0;
    const bool diag_memory = (diag_feature_mask & DSP_CHARACTER_FEATURE_MEMORY) != 0;
    const bool diag_spectral = (diag_feature_mask & DSP_CHARACTER_FEATURE_SPECTRAL) != 0;

    /* Clear memory-only state once when its diagnostic path is bypassed. */
    if (s_character_memory_feature_active && !diag_memory) {
        for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
            character_stage_state_t *ss = &s_character_state.stage[stage];
            memset(ss->charge, 0, sizeof(ss->charge));
            memset(ss->coupling, 0, sizeof(ss->coupling));
            memset(ss->bias, 0, sizeof(ss->bias));
        }
        s_character_state.supply_envelope = 0.0f;
    }
    s_character_memory_feature_active = diag_memory;

    const size_t frames =
        len_bytes / 4;


    const size_t required =
        len_bytes;


    /*
     * Scratch buffer.
     */

    if (!s_scratch || required > s_scratch_len) {
        ESP_LOGE(
            TAG,
            "PCM frame exceeds preallocated DSP scratch: %u > %u",
            (unsigned)required,
            (unsigned)s_scratch_len
        );
        return pcm_in;
    }


    const int16_t *in =
        (const int16_t *)pcm_in;


    int16_t *out =
        (int16_t *)s_scratch;


    /*
     * Envelope follower coefficients.
     */
    float env_attack =
        exciter_attack_alpha();

    float env_release =
        exciter_release_alpha();

    const float character_control_smooth = 1.0f - expf(
        -1.0f / (0.012f * DSP_SAMPLE_RATE_HZ));

    /* Per-call profiling: cycle-counter reads are much lighter than timers. */
    uint32_t character_stage_cycles[CHARACTER_STAGE_COUNT] = {0};

    for (size_t i = 0;
         i < frames;
         i++) {

        float l =
            (float)in[i * 2] /
            32768.0f;


        float r =
            (float)in[i * 2 + 1] /
            32768.0f;


        /*
         * ----------------------------------------------------
         * Original 10-band PEQ + shelf
         * ----------------------------------------------------
         */

        float yl =
            l;

        float yr =
            r;


        for (uint8_t stage = 0;
             stage < num_stages;
             stage++) {

            const biquad_coef_t *c =
                &coefs[stage];


            /*
             * Left
             */

            yl =
                biquad_process_sample(
                    yl,
                    c,
                    &s_state[stage][0]
                );


            /*
             * Right
             */

            yr =
                biquad_process_sample(
                    yr,
                    c,
                    &s_state[stage][1]
                );
        }


        /*
         * ----------------------------------------------------
         * Analog Character V4.1 — three-stage stateful preamp
         * ----------------------------------------------------
         * The original signal remains the dry backbone. Each stage filters a
         * frequency region, creates a small ADAA nonlinear residual, low-pass
         * limits that residual, then returns it in parallel. No audio sample
         * is fed back; memory lives in bounded envelope/coupling/supply states.
         */
        {
            character_state_t *cs = &s_character_state;
            const float smooth = character_control_smooth;


            const float tw = (character_enabled && diag_warmth)
                ? character.warmth_pct * 0.01f : 0.0f;
            const float td = (character_enabled && diag_detail)
                ? character.detail_pct * 0.01f : 0.0f;
            const float tr = (character_enabled && diag_drive)
                ? character.drive_pct * 0.01f : 0.0f;
            const float ts = (character_enabled && diag_softness)
                ? character.softness_pct * 0.01f : 0.0f;
            const float tm = (character_enabled && diag_memory)
                ? character.memory_pct * 0.01f : 0.0f;
            /* 50% is the neutral stage-balance point when Spectral is bypassed. */
            const float tsp = (character_enabled && diag_spectral)
                ? character.spectral_pct * 0.01f : 0.5f;
            const float ta = (character_enabled && diag_air)
                ? (db_to_linear(character.air_pct * 0.015f) - 1.0f) : 0.0f;

            cs->warmth += smooth * (tw - cs->warmth);
            cs->detail += smooth * (td - cs->detail);
            cs->drive += smooth * (tr - cs->drive);
            cs->softness += smooth * (ts - cs->softness);
            cs->memory += smooth * (tm - cs->memory);
            cs->spectral += smooth * (tsp - cs->spectral);
            cs->air_gain += smooth * (ta - cs->air_gain);

            const bool nonlinear_control_active =
                (cs->warmth > 0.00002f) ||
                (cs->detail > 0.00002f);
            const bool air_control_active =
                fabsf(cs->air_gain) > 0.000002f;
            const bool character_mix_active =
                ((diag_stage_mask != 0) && nonlinear_control_active) ||
                air_control_active;

            /* Do not run the nonlinear network if it cannot contribute audio. */
            if (character_mix_active || cs->running) {
                const bool memory_active = diag_memory && (cs->memory > 0.00001f);

                float supply_level = 0.0f;
                if (memory_active) {
                    const float program_level = 0.5f * (fabsf(yl) + fabsf(yr));
                    const float supply_alpha =
                        (program_level > cs->supply_envelope)
                        ? character_filters.supply_attack_alpha
                        : character_filters.supply_release_alpha;
                    cs->supply_envelope += supply_alpha *
                        (program_level - cs->supply_envelope);
                    supply_level = clampf_local(
                        (cs->supply_envelope - 0.025f) / 0.20f, 0.0f, 1.0f);
                }

                const float effective_softness =
                    diag_softness ? cs->softness : 0.0f;
                const float supply_sag = memory_active
                    ? 0.055f * cs->memory *
                        (0.35f + 0.65f * effective_softness) * supply_level
                    : 0.0f;
                const float common_drive = diag_drive
                    ? (0.90f + 1.90f * cs->drive) * (1.0f - supply_sag)
                    : 1.0f;

                const float spectral = diag_spectral ? cs->spectral : 0.5f;
                const float stage_mix[CHARACTER_STAGE_COUNT] = {
                    0.190f * cs->warmth * (1.15f - 0.30f * spectral),
                    (0.105f * cs->warmth + 0.075f * cs->detail) *
                        (0.90f + 0.20f * spectral),
                    (0.020f * cs->warmth + 0.065f * cs->detail) *
                        (0.55f + 0.90f * spectral)
                };
                const float base_bias[CHARACTER_STAGE_COUNT] = {
                    0.020f + 0.105f * cs->warmth,
                    0.010f + 0.065f * cs->warmth + 0.018f * cs->detail,
                    0.004f + 0.022f * cs->warmth + 0.014f * cs->detail
                };
                const float stage_drive[CHARACTER_STAGE_COUNT] = {
                    common_drive * 0.92f,
                    common_drive,
                    common_drive * 0.55f
                };

                uint8_t active_stage_mask = 0;
                for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
                    const uint32_t bit = 1U << stage;
                    if ((diag_stage_mask & bit) && stage_mix[stage] > 0.0000001f)
                        active_stage_mask |= (uint8_t)bit;
                }

                /* Reset a stage only on the transition to bypass, not per sample. */
                const uint8_t stages_just_disabled =
                    s_character_active_stage_mask & (uint8_t)(~active_stage_mask);
                for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
                    if (stages_just_disabled & (1U << stage)) {
                        memset(&cs->stage[stage], 0, sizeof(cs->stage[stage]));
                    }
                }
                s_character_active_stage_mask = active_stage_mask;

                const float character_dry[2] = { yl, yr };
                float character_residual_sum[2] = { 0.0f, 0.0f };

                for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
                    const uint32_t stage_bit = 1U << stage;
                    if (!(active_stage_mask & stage_bit))
                        continue;

                    const uint32_t stage_start_cycles = esp_cpu_get_cycle_count();
                    character_stage_state_t *ss = &cs->stage[stage];
                    const character_stage_filter_t *fc =
                        &character_filters.stage[stage];
                    float band[2];

                    for (unsigned ch = 0; ch < 2; ++ch) {
                        band[ch] = biquad_process_sample(
                            character_dry[ch], &fc->input_hpf, &ss->input_hpf[ch]);
                        band[ch] = biquad_process_sample(
                            band[ch], &fc->input_lpf, &ss->input_lpf[ch]);

                        const float abs_band = fabsf(band[ch]);
                        const float env_alpha = (abs_band > ss->envelope[ch])
                            ? fc->env_attack_alpha : fc->env_release_alpha;
                        ss->envelope[ch] += env_alpha *
                            (abs_band - ss->envelope[ch]);

                        float charge_level = 0.0f;
                        float bias = 0.0f;
                        if (memory_active) {
                            const float charge_alpha = (abs_band > ss->charge[ch])
                                ? fc->charge_attack_alpha : fc->charge_release_alpha;
                            ss->charge[ch] += charge_alpha *
                                (abs_band - ss->charge[ch]);
                            ss->coupling[ch] += fc->coupling_alpha *
                                (band[ch] - ss->coupling[ch]);
                            ss->coupling[ch] = clampf_local(
                                ss->coupling[ch], -0.12f, 0.12f);

                            const float level = clampf_local(
                                (ss->envelope[ch] - 0.012f) / 0.16f, 0.0f, 1.0f);
                            charge_level = clampf_local(
                                (ss->charge[ch] - 0.008f) / 0.14f, 0.0f, 1.0f);
                            const float bias_target = clampf_local(
                                base_bias[stage] + cs->memory *
                                    (0.030f * level + 0.050f * charge_level +
                                     0.025f * ss->coupling[ch]),
                                -0.16f, 0.18f);
                            const float bias_alpha =
                                (bias_target > ss->bias[ch])
                                ? fc->bias_attack_alpha : fc->bias_release_alpha;
                            ss->bias[ch] += bias_alpha *
                                (bias_target - ss->bias[ch]);
                            bias = ss->bias[ch];
                        }

                        const float drive_now = fmaxf(0.25f, stage_drive[stage]);
                        const float peak_now = clampf_local(
                            ss->envelope[ch] * 1.57079632679f, 0.0f, 0.95f);
                        const float asymmetry = clampf_local(
                            0.62f + 1.35f * bias, 0.35f, 0.95f);
                        const float level_scale = diag_softness
                            ? (1.0f - 0.18f * cs->softness * supply_level)
                            : 1.0f;
                        const float mix = stage_mix[stage] * level_scale;
                        const float even_gain = mix * drive_now * asymmetry;
                        const float odd_shape = diag_detail
                            ? (0.075f + 0.11f * cs->detail + 0.055f * charge_level)
                            : 0.0f;
                        const float odd_gain = mix * drive_now * odd_shape;

                        float residual;
                        if (ss->adaa_valid[ch]) {
                            const float even_avg =
                                0.5f * (even_gain + ss->prev_even_gain[ch]);
                            const float odd_avg =
                                0.5f * (odd_gain + ss->prev_odd_gain[ch]);
                            residual = character_adaa_poly_residual(
                                band[ch], ss->prev_x[ch],
                                peak_now, ss->prev_peak[ch],
                                even_avg, odd_avg);
                        } else {
                            const float peak2 = peak_now * peak_now;
                            const float h2 = band[ch] * band[ch] - 0.5f * peak2;
                            const float h3 = band[ch] * band[ch] * band[ch] -
                                0.75f * peak2 * band[ch];
                            residual = even_gain * h2 + odd_gain * h3;
                            ss->adaa_valid[ch] = true;
                        }
                        ss->prev_x[ch] = band[ch];
                        ss->prev_even_gain[ch] = even_gain;
                        ss->prev_odd_gain[ch] = odd_gain;
                        ss->prev_peak[ch] = peak_now;

                        residual = biquad_process_sample(
                            residual, &fc->residual_hpf, &ss->residual_hpf[ch]);
                        if (stage == 2) {
                            residual = biquad_process_sample(
                                residual, &fc->residual_lpf, &ss->residual_lpf[ch]);
                        }
                        character_residual_sum[ch] += residual;
                    }

                    character_stage_cycles[stage] +=
                        (uint32_t)(esp_cpu_get_cycle_count() - stage_start_cycles);
                }

                yl = character_dry[0] + character_residual_sum[0];
                yr = character_dry[1] + character_residual_sum[1];

                /* Air is a separate linear shelf and can be bypassed independently. */
                if (diag_air && fabsf(cs->air_gain) > 0.000002f) {
                    for (unsigned ch = 0; ch < 2; ++ch) {
                        float *sample = (ch == 0) ? &yl : &yr;
                        cs->air_lp[ch] += character_filters.air_alpha *
                            (*sample - cs->air_lp[ch]);
                        *sample += (*sample - cs->air_lp[ch]) * cs->air_gain;
                    }
                } else if (!diag_air) {
                    cs->air_lp[0] = 0.0f;
                    cs->air_lp[1] = 0.0f;
                }

                cs->running = character_mix_active;
                if (!cs->running) {
                    memset(cs->stage, 0, sizeof(cs->stage));
                    cs->supply_envelope = 0.0f;
                    cs->air_lp[0] = 0.0f;
                    cs->air_lp[1] = 0.0f;
                    s_character_active_stage_mask = 0;
                }
            }
        }

        /*
         * ----------------------------------------------------
         * Bass Harmonic Exciter
         * ----------------------------------------------------
         *
         * PEQ/Shelf
         *      ↓
         * Mono bass
         *      ↓
         * Butterworth LPF
         *      ↓
         * Envelope activity / peak estimate
         *      ↓
         * Chebyshev T2 / T3
         *      ↓
         * Separate H2/H3 filtering
         *      ↓
         * Harmonic mix
         *      ↓
         * Dry L/R + harmonic
         *
         * Master is applied afterwards.
         * ----------------------------------------------------
         */

        /* Exciter is a strictly optional additive branch. Its enable flag
         * never gates the PEQ chain above or the master/limiter below. */
        if (exc_enabled &&
            (exciter.h2_pct > 0.0f ||
             exciter.h3_pct > 0.0f)) {

            /*
             * -----------------------------------------------
             * 1. Mono bass
             * -----------------------------------------------
             */
            float bass =
                0.5f *
                (yl + yr);


            /*
             * -----------------------------------------------
             * 2. Bass crossover LPF
             * -----------------------------------------------
             */
            float low =
                biquad_process_sample(
                    bass,
                    &exc_filters.bass_lpf,
                    &s_exciter_state.bass_lpf
                );


            /*
             * -----------------------------------------------
             * 3. Gentle envelope activity control
             * -----------------------------------------------
             *
             * Do NOT normalize low to +/-1 anymore.  Full
             * normalization makes quiet bass passages produce an
             * almost constant synthetic harmonic layer and can
             * make the exciter sound like an added bass boost.
             *
             * The envelope is now only used as a soft activity
             * gate.  Harmonics retain the real bass dynamics.
             * -----------------------------------------------
             */
            float abs_low = fabsf(low);
            float env = s_exciter_state.envelope;

            if (abs_low > env) {
                env += env_attack * (abs_low - env);
            } else {
                env += env_release * (abs_low - env);
            }

            s_exciter_state.envelope = env;

            float activity =
                clampf_local(
                    (env - EXC_ENV_GATE) /
                    EXC_ENV_SOFT_START,
                    0.0f,
                    1.0f
                );

            /*
             * -----------------------------------------------
             * 4. Chebyshev harmonic generation
             * -----------------------------------------------
             *
             * Raw bass is intentionally used here.  This preserves
             * level-dependent harmonic density instead of forcing
             * every quiet note to full nonlinear drive.
             *
             * T2 = 2x² - A²
             * T3 = 4x³ - 3A²x
             * A is an amplitude estimate from the bass envelope.
             *
             * Residual DC / fundamental leakage is further reduced by
             * the dedicated HPF stages immediately afterwards.
             * -----------------------------------------------
             */
            float x = clampf_local(
                low * EXC_NONLINEAR_DRIVE,
                -1.0f,
                1.0f
            );

            /*
             * Estimate the driven sine peak from the smoothed mean-|x|
             * envelope. This makes T2/T3 amplitude-aware instead of
             * assuming every sample is a full-scale normalized sine.
             */
            float peak = clampf_local(
                env * 1.57079632679f * EXC_NONLINEAR_DRIVE,
                0.0f,
                1.0f
            );

            /* T2 = 2*x^2 - A^2: cancels the DC term for a sine. */
            float h2 = 2.0f * x * x - peak * peak;

            /* T3 = 4*x^3 - 3*A^2*x: cancels its fundamental component. */
            float h3 = 4.0f * x * x * x - 3.0f * peak * peak * x;

            /*
             * Activity gate only.  No envelope normalization.
             */
            h2 *= activity;
            h3 *= activity;

            /*
             * -----------------------------------------------
             * 6. H2 branch
             *
             * HPF 65 Hz
             * LPF 160 Hz
             * -----------------------------------------------
             */
            h2 =
                biquad_process_sample(
                    h2,
                    &exc_filters.h2_hpf,
                    &s_exciter_state.h2_hpf
                );


            h2 =
                biquad_process_sample(
                    h2,
                    &exc_filters.h2_lpf,
                    &s_exciter_state.h2_lpf
                );


            /*
             * -----------------------------------------------
             * 7. H3 branch
             *
             * HPF 65 Hz
             * LPF 180 Hz
             * -----------------------------------------------
             */
            h3 =
                biquad_process_sample(
                    h3,
                    &exc_filters.h3_hpf,
                    &s_exciter_state.h3_hpf
                );


            h3 =
                biquad_process_sample(
                    h3,
                    &exc_filters.h3_lpf,
                    &s_exciter_state.h3_lpf
                );


            /*
             * -----------------------------------------------
             * 8. Harmonic mix
             * -----------------------------------------------
             *
             * 0.06 = 6%
             * 0.02 = 2%
             */
            float harmonic =
                exciter.h2_pct * h2 +
                exciter.h3_pct * h3;


            /*
             * -----------------------------------------------
             * 9. Add equally to L/R
             * -----------------------------------------------
             *
             * Bass remains mono.
             * Stereo image from the original signal is
             * untouched.
             */
            yl +=
                harmonic;

            yr +=
                harmonic;
        }




        /*
         * ----------------------------------------------------
         * Master
         * ----------------------------------------------------
         */

        yl *=
            master_linear;

        yr *=
            master_linear;


        /*
         * Soft-knee peak control. Linear below 0.94, asymptotically
         * approaches 0.985 above it; avoids the old hard-clipping edge.
         */
        const float limiter_knee = 0.94f;
        const float limiter_range = 0.045f;

        float abs_l = fabsf(yl);
        if (abs_l > limiter_knee) {
            float over = abs_l - limiter_knee;
            float limited = limiter_knee +
                over / (1.0f + over / limiter_range);
            yl = copysignf(limited, yl);
        }

        float abs_r = fabsf(yr);
        if (abs_r > limiter_knee) {
            float over = abs_r - limiter_knee;
            float limited = limiter_knee +
                over / (1.0f + over / limiter_range);
            yr = copysignf(limited, yr);
        }

        yl = clampf_local(yl, -0.985f, 0.985f);
        yr = clampf_local(yr, -0.985f, 0.985f);


        out[i * 2] =
            (int16_t)(
                yl * 32768.0f
            );


        out[i * 2 + 1] =
            (int16_t)(
                yr * 32768.0f
            );
    }


    if (character_enabled) {
        const uint32_t cpu_mhz =
            (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ > 0)
            ? CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ : 240U;

        for (unsigned stage = 0; stage < CHARACTER_STAGE_COUNT; ++stage) {
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

    return s_scratch;
}