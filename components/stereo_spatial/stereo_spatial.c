#include "stereo_spatial.h"
#include "esp_attr.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"

#include "dsps_biquad.h"
#include "dsps_biquad_gen.h"

static const char *TAG = "stereo_spatial";

/* ============================================================
 * V3.2 TUNING
 * ============================================================ */

#define SS_SIDE_HPF_FREQ       180.0f

#define SS_APF_FREQ_1          1100.0f
#define SS_APF_FREQ_2          3000.0f
#define SS_APF_FREQ_3          7200.0f

#define SS_DECOR_DIRECT        0.94f
#define SS_DECOR_DIFF          0.06f

#define SS_DENORMAL_THRESHOLD  1.0e-15f

#define SS_EQ_BANDS            5
#define SS_APF_STAGES          3
#define SS_FILTER_SLOTS        2

/*
 * DSP block buffer.
 *
 * AirPlay block size is not assumed to be fixed.
 * These are grown dynamically only if a larger block arrives.
 *
 * IMPORTANT:
 * This allocation is NOT performed inside the normal
 * processing path once the required size is known.
 */
#define SS_INITIAL_BLOCK_FRAMES    512

/* ============================================================
 * INTERNAL TYPES
 * ============================================================ */

/*
 * ESP-DSP biquad coefficient format:
 *
 *   b0 b1 b2 a1 a2
 *
 * a0 is assumed to be 1.
 */
typedef struct {
    float coeffs[5];
} biquad_coeff_t;

/*
 * ESP-DSP Direct Form II state:
 *
 *   w[0], w[1]
 */
typedef struct {
    float w[2];
} biquad_state_t;

typedef struct {
    stereo_spatial_config_t config;

    biquad_coeff_t side_hpf;

    biquad_coeff_t side_eq[SS_EQ_BANDS];

    biquad_coeff_t apf[SS_APF_STAGES];

} stereo_spatial_filter_set_t;


/*
 * Runtime state is deliberately OUTSIDE the double-buffered
 * coefficient sets.
 *
 * The audio task owns these states.
 * Preset changes therefore do not reset filter history.
 */
typedef struct {
    biquad_state_t side_hpf;

    biquad_state_t side_eq[SS_EQ_BANDS];

    biquad_state_t apf[SS_APF_STAGES];

} stereo_spatial_runtime_t;


struct stereo_spatial_ctx {

    uint32_t sample_rate;

    /*
     * Double-buffered coefficient sets.
     *
     * Audio reads ONLY active_filter_index.
     * Control task writes ONLY inactive slot.
     */
    stereo_spatial_filter_set_t filter_set[SS_FILTER_SLOTS];

    _Atomic uint32_t active_filter_index;

    /*
     * Runtime filter history.
     * Owned exclusively by audio processing.
     */
    stereo_spatial_runtime_t runtime;

    /*
     * Control-side lock only.
     *
     * NEVER taken by stereo_spatial_process_i16().
     */
    SemaphoreHandle_t config_lock;

    /*
     * Block working buffers.
     *
     * L/R are needed only temporarily.
     * M is stored in work_l.
     * S is stored in work_r.
     */
    float *work_l;
    float *work_r;
    float *work_side_decor;


    size_t work_capacity;

};


/* ============================================================
 * SMALL HELPERS
 * ============================================================ */

static inline float flush_denormal(float x)
{
    return (fabsf(x) < SS_DENORMAL_THRESHOLD) ? 0.0f : x;
}


static inline int16_t float_to_i16(float x)
{
    if (x > 32767.0f)
        return 32767;

    if (x < -32768.0f)
        return -32768;

    /*
     * lrintf gives normal rounding rather than truncation.
     */
    return (int16_t)lrintf(x);
}


/* ============================================================
 * COEFFICIENT GENERATION
 * ============================================================ */

static bool build_hpf(
    float *coeffs,
    float sample_rate,
    float freq,
    float q)
{
    if (!coeffs || sample_rate <= 0.0f)
        return false;

    /*
     * ESP-DSP expects normalized frequency:
     *
     *      f / Fs
     *
     * in the range 0 .. 0.5
     */
    const float normalized = freq / sample_rate;

    if (normalized <= 0.0f || normalized >= 0.5f)
        return false;

    return dsps_biquad_gen_hpf_f32(
        coeffs,
        normalized,
        q
    ) == ESP_OK;
}


static bool build_peq(
    float *coeffs,
    float sample_rate,
    float freq,
    float gain_db,
    float q)
{
    if (!coeffs || sample_rate <= 0.0f)
        return false;

    const float w0 =
        2.0f * (float)M_PI * freq / sample_rate;

    const float alpha =
        sinf(w0) / (2.0f * q);

    const float A =
        powf(10.0f, gain_db / 40.0f);

    const float cos_w0 =
        cosf(w0);

    const float b0 =
        1.0f + alpha * A;

    const float b1 =
        -2.0f * cos_w0;

    const float b2 =
        1.0f - alpha * A;

    const float a0 =
        1.0f + alpha / A;

    const float a1 =
        -2.0f * cos_w0;

    const float a2 =
        1.0f - alpha / A;

    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = a1 / a0;
    coeffs[4] = a2 / a0;

    return true;
}


static bool build_apf(
    float *coeffs,
    float sample_rate,
    float freq,
    float q)
{
    if (!coeffs || sample_rate <= 0.0f)
        return false;

    const float normalized = freq / sample_rate;

    if (normalized <= 0.0f || normalized >= 0.5f)
        return false;

    /*
     * 360-degree all-pass.
     *
     * Magnitude remains approximately flat while phase is
     * changed. This is exactly what we want for subtle
     * decorrelation.
     */
    return dsps_biquad_gen_allpass360_f32(
        coeffs,
        normalized,
        q
    ) == ESP_OK;
}


/* ============================================================
 * BUILD COMPLETE FILTER SET
 * ============================================================ */

static bool build_filter_set(
    stereo_spatial_filter_set_t *set,
    uint32_t sample_rate,
    const stereo_spatial_config_t *config)
{
    if (!set || !config || sample_rate == 0)
        return false;

    memset(set, 0, sizeof(*set));

    set->config = *config;

    /*
     * OFF still gets valid unity-ish filter data.
     * They simply won't be executed.
     */

    /* --------------------------------------------------------
     * Side HPF
     * -------------------------------------------------------- */

    if (!build_hpf(
            set->side_hpf.coeffs,
            (float)sample_rate,
            SS_SIDE_HPF_FREQ,
            0.707f))
    {
        ESP_LOGE(TAG, "Failed to build Side HPF");
        return false;
    }


    /* --------------------------------------------------------
     * Side EQ
     *
     * Frequency/Q structure deliberately unchanged from V3.1.
     *
     * NATURAL:
     *   500 Hz   +1.5 dB
     *   1.2 kHz  +2.0 dB
     *   3 kHz    +2.5 dB
     *   7 kHz    +2.0 dB
     *   14 kHz   +1.2 dB
     *
     * WIDE:
     *   500 Hz   +2.5 dB
     *   1.2 kHz  +3.5 dB
     *   3 kHz    +4.0 dB
     *   7 kHz    +3.5 dB
     *   14 kHz   +2.0 dB
     * -------------------------------------------------------- */

    static const float eq_freq[SS_EQ_BANDS] = {
        500.0f,
        1200.0f,
        3000.0f,
        7000.0f,
        14000.0f
    };

    static const float eq_q[SS_EQ_BANDS] = {
        0.85f,
        0.95f,
        1.05f,
        0.90f,
        0.75f
    };

    static const float natural_gain[SS_EQ_BANDS] = {
        1.5f,
        2.0f,
        2.5f,
        2.0f,
        1.2f
    };

    static const float wide_gain[SS_EQ_BANDS] = {
        2.5f,
        3.5f,
        4.0f,
        3.5f,
        2.0f
    };

    const float *gain_table = natural_gain;

    if (config->mode == STEREO_SPATIAL_MODE_WIDE)
        gain_table = wide_gain;


    for (int i = 0; i < SS_EQ_BANDS; ++i) {

        float gain = 0.0f;

        if (config->enable_width_eq)
            gain = gain_table[i];

        /*
         * Even when EQ is disabled, create a valid coefficient
         * set. Processing simply won't be called.
         */
        if (!build_peq(
                set->side_eq[i].coeffs,
                (float)sample_rate,
                eq_freq[i],
                gain,
                eq_q[i]))
        {
            ESP_LOGE(TAG, "Failed to build Side EQ %d", i);
            return false;
        }
    }


    /* --------------------------------------------------------
     * Decorrelator
     * -------------------------------------------------------- */

    if (!build_apf(
            set->apf[0].coeffs,
            (float)sample_rate,
            SS_APF_FREQ_1,
            0.70f))
    {
        ESP_LOGE(TAG, "Failed to build APF 1");
        return false;
    }

    if (!build_apf(
            set->apf[1].coeffs,
            (float)sample_rate,
            SS_APF_FREQ_2,
            0.75f))
    {
        ESP_LOGE(TAG, "Failed to build APF 2");
        return false;
    }

    if (!build_apf(
            set->apf[2].coeffs,
            (float)sample_rate,
            SS_APF_FREQ_3,
            0.80f))
    {
        ESP_LOGE(TAG, "Failed to build APF 3");
        return false;
    }

    return true;
}




/* ============================================================
 * INIT
 * ============================================================ */

stereo_spatial_ctx_t *
stereo_spatial_init(uint32_t sample_rate)
{
    if (sample_rate == 0)
        return NULL;

    stereo_spatial_ctx_t *ctx =
        calloc(1, sizeof(stereo_spatial_ctx_t));

    if (!ctx)
        return NULL;

    ctx->sample_rate = sample_rate;

    ctx->config_lock = xSemaphoreCreateMutex();

    if (!ctx->config_lock) {
        free(ctx);
        return NULL;
    }


    stereo_spatial_config_t default_config = {
        .mode = STEREO_SPATIAL_MODE_OFF,
        .enable_width_eq = false,
        .enable_decorrelator = false
    };


    /*
     * Build BOTH slots before publishing the context.
     */
    if (!build_filter_set(
            &ctx->filter_set[0],
            sample_rate,
            &default_config))
    {
        vSemaphoreDelete(ctx->config_lock);
        free(ctx);
        return NULL;
    }

    if (!build_filter_set(
            &ctx->filter_set[1],
            sample_rate,
            &default_config))
    {
        vSemaphoreDelete(ctx->config_lock);
        free(ctx);
        return NULL;
    }


    memset(
        &ctx->runtime,
        0,
        sizeof(ctx->runtime)
    );


    ctx->work_capacity = SS_INITIAL_BLOCK_FRAMES;

    ctx->work_l = calloc(
        ctx->work_capacity,
        sizeof(float)
    );

    ctx->work_r = calloc(
        ctx->work_capacity,
        sizeof(float)
    );

    ctx->work_side_decor = calloc(
        ctx->work_capacity,
        sizeof(float)
    );

    if (!ctx->work_l || !ctx->work_r || !ctx->work_side_decor) {

        free(ctx->work_l);
        free(ctx->work_r);
        free(ctx->work_side_decor);

        vSemaphoreDelete(ctx->config_lock);
        free(ctx);

        return NULL;
    }


    atomic_init(
        &ctx->active_filter_index,
        0
    );


    ESP_LOGI(
        TAG,
        "Stereo Spatial V3.2 init: %lu Hz",
        (unsigned long)sample_rate
    );

    return ctx;
}


/* ============================================================
 * DESTROY
 * ============================================================ */

void stereo_spatial_destroy(
    stereo_spatial_ctx_t *ctx)
{
    if (!ctx)
        return;

    free(ctx->work_l);
    free(ctx->work_r);
    free(ctx->work_side_decor);

    if (ctx->config_lock)
        vSemaphoreDelete(ctx->config_lock);

    free(ctx);
}


/* ============================================================
 * GET CONFIG
 * ============================================================ */

void stereo_spatial_get_config(
    const stereo_spatial_ctx_t *ctx,
    stereo_spatial_config_t *cfg)
{
    if (!ctx || !cfg)
        return;

    /*
     * Active filter set is immutable while active.
     *
     * Acquire guarantees visibility of the fully-built
     * coefficient set after atomic publication.
     */
    uint32_t index =
        atomic_load_explicit(
            &ctx->active_filter_index,
            memory_order_acquire
        );

    *cfg = ctx->filter_set[index].config;
}


/* ============================================================
 * SET CONFIG
 * ============================================================ */

void stereo_spatial_set_config(
    stereo_spatial_ctx_t *ctx,
    const stereo_spatial_config_t *config)
{
    if (!ctx || !config)
        return;

    if (xSemaphoreTake(
            ctx->config_lock,
            portMAX_DELAY) != pdTRUE)
    {
        return;
    }


    /*
     * Current active slot.
     */
    uint32_t active =
        atomic_load_explicit(
            &ctx->active_filter_index,
            memory_order_acquire
        );

    /*
     * Only write to inactive slot.
     */
    uint32_t inactive = active ^ 1U;


    /*
     * Build everything before publishing.
     *
     * Audio task can continue processing the current slot.
     */
    bool ok = build_filter_set(
        &ctx->filter_set[inactive],
        ctx->sample_rate,
        config
    );


    if (ok) {

        /*
         * Publish COMPLETE filter set.
         *
         * Audio block already in progress continues with old
         * coefficients. Next block sees the new set.
         */
        atomic_store_explicit(
            &ctx->active_filter_index,
            inactive,
            memory_order_release
        );

        ESP_LOGI(
            TAG,
            "Preset swapped: mode=%d EQ=%d decor=%d",
            config->mode,
            config->enable_width_eq,
            config->enable_decorrelator
        );
    }
    else {
        ESP_LOGE(
            TAG,
            "Failed to build new filter set"
        );
    }


    xSemaphoreGive(ctx->config_lock);
}


/* ============================================================
 * BLOCK BIQUAD HELPER
 * ============================================================ */

static inline void process_biquad_block(
    float *buffer,
    int frames,
    const biquad_coeff_t *coeff,
    biquad_state_t *state)
{
    /*
     * ESP-DSP keeps state across calls through w[2].
     *
     * dsps_biquad_f32 is mapped to the architecture-specific
     * implementation. On ESP32-S3 this can use the AES3
     * optimized implementation.
     */
    (void)dsps_biquad_f32(
        buffer,
        buffer,
        frames,
        (float *)coeff->coeffs,
        state->w
    );
}


/* ============================================================
 * MAIN AUDIO PROCESSOR
 * ============================================================ */

void IRAM_ATTR stereo_spatial_process_i16(
    stereo_spatial_ctx_t *ctx,
    int16_t *buffer,
    size_t frames)
{
    if (!ctx || !buffer || frames == 0)
        return;


    /*
     * IMPORTANT:
     *
     * No mutex.
     * No allocation.
     * No logging.
     *
     * One atomic load per audio block.
     */
    uint32_t active =
        atomic_load_explicit(
            &ctx->active_filter_index,
            memory_order_acquire
        );

    const stereo_spatial_filter_set_t *filter =
        &ctx->filter_set[active];

    const stereo_spatial_config_t *config =
        &filter->config;


    if (config->mode == STEREO_SPATIAL_MODE_OFF)
        return;


    /*
     * The audio engine normally gives blocks <= 512 frames.
     *
     * If an unexpectedly large block arrives, this function
     * currently avoids reallocating in the audio path.
     */
    if (frames > ctx->work_capacity) {

        /*
         * Do NOT realloc from the audio callback.
         *
         * Safest behavior:
         * process nothing rather than introduce an unpredictable
         * allocation stall/dropout.
         *
         * Normally this branch should never execute.
         */
        ESP_LOGW(
            TAG,
            "Block %u exceeds DSP buffer %u",
            (unsigned)frames,
            (unsigned)ctx->work_capacity
        );

        return;
    }


    float *mid  = ctx->work_l;
    float *side = ctx->work_r;


    /* ========================================================
     * i16 -> M/S float
     * ======================================================== */

    const float inv32768 = 1.0f / 32768.0f;

    for (size_t i = 0; i < frames; ++i) {

        const float l =
            (float)buffer[i * 2 + 0] * inv32768;

        const float r =
            (float)buffer[i * 2 + 1] * inv32768;

        mid[i]  = 0.5f * (l + r);
        side[i] = 0.5f * (l - r);
    }


    /* ========================================================
     * SIDE HPF
     *
     * Bass remains essentially centered.
     * ======================================================== */

    process_biquad_block(
        side,
        (int)frames,
        &filter->side_hpf,
        &ctx->runtime.side_hpf
    );


    /* ========================================================
     * SIDE EQ
     * ======================================================== */

    if (config->enable_width_eq) {

        for (int band = 0;
             band < SS_EQ_BANDS;
             ++band)
        {
            process_biquad_block(
                side,
                (int)frames,
                &filter->side_eq[band],
                &ctx->runtime.side_eq[band]
            );
        }
    }


    /* ========================================================
     * PHASE DECORRELATION
     * ======================================================== */

    if (config->enable_decorrelator) {

        /*
        * Keep original Side untouched.
        *
        * side          = direct Side
        * side_decor    = phase-decorrelated Side
        */
        memcpy(
            ctx->work_side_decor,
            side,
            frames * sizeof(float)
        );

        /*
        * 3-stage all-pass decorrelator.
        *
        * Magnitude remains approximately flat.
        * Main effect is frequency-dependent phase shift.
        */
        for (int stage = 0;
            stage < SS_APF_STAGES;
            ++stage)
        {
            process_biquad_block(
                ctx->work_side_decor,
                (int)frames,
                &filter->apf[stage],
                &ctx->runtime.apf[stage]
            );
        }
    }


    /* ========================================================
     * M/S -> L/R
     * ======================================================== */

    for (size_t i = 0; i < frames; ++i) {

        float m = mid[i];
        float s = side[i];

        m = flush_denormal(m);
        s = flush_denormal(s);

        /*
        * Natural decorrelation:
        *
        * 94% original Side
        *  6% phase-decorrelated Side
        *
        * Bass remains controlled because Side already passed
        * through the 180 Hz HPF.
        */
        if (config->enable_decorrelator) {

            float sd =
                flush_denormal(ctx->work_side_decor[i]);

            s =
                SS_DECOR_DIRECT * s +
                SS_DECOR_DIFF   * sd;
        }

        float l = m + s;
        float r = m - s;

        /*
        * Safety clip.
        */
        if (l > 0.999969f)
            l = 0.999969f;
        else if (l < -1.0f)
            l = -1.0f;

        if (r > 0.999969f)
            r = 0.999969f;
        else if (r < -1.0f)
            r = -1.0f;

        buffer[i * 2 + 0] =
            float_to_i16(l * 32768.0f);

        buffer[i * 2 + 1] =
            float_to_i16(r * 32768.0f);
    }
}