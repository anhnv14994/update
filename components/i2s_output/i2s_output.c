#include "i2s_output.h"

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "i2s_output";

#define I2S_VOLUME_BUFFER_SAMPLES 1024U /* 512 stereo frames / 2048 bytes */
#define I2S_VOLUME_RAMP_ALPHA       0.002830453811f /* 8 ms at 44.1 kHz, once per frame */
#define I2S_TRANSITION_RAMP_SAMPLES 441U /* 10 ms at 44.1 kHz */

static _Atomic uint32_t target_volume_q = 50000U; /* RAOP startup default: 0.05 */
static float current_volume = 0.05f;             /* Owned by the I2S output task. */
static _Atomic uint32_t target_transition_q = 1000000U;
static _Atomic bool transition_restart_requested = false;
static float current_transition_gain = 1.0f;      /* Owned by the I2S output task. */
static int16_t volume_buffer[I2S_VOLUME_BUFFER_SAMPLES];
static uint32_t dither_rng = 0x6D2B79F5U;

static i2s_chan_handle_t tx_handle = NULL;

static inline float dither_uniform(void)
{
    dither_rng = dither_rng * 1664525U + 1013904223U;
    return (float)(dither_rng >> 8) * (1.0f / 16777216.0f);
}

static inline int16_t scale_and_quantize(int16_t sample, float volume)
{
    /* Sub-2-LSB digital silence floor: do not dither inaudible noise-floor residue. */
    if ((sample >= -1 && sample <= 1) || volume <= 0.0f)
        return 0;

    float scaled = (float)sample * volume;

    /* TPDF dither of +/- 1 LSB at the final PCM16 conversion. */
    if (volume < 0.999999f)
        scaled += dither_uniform() - dither_uniform();

    if (scaled > 32767.0f)
        scaled = 32767.0f;
    else if (scaled < -32768.0f)
        scaled = -32768.0f;

    const int32_t rounded = (int32_t)(
        scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
    return (int16_t)rounded;
}

void i2s_output_set_volume(float volume)
{
    if (!isfinite(volume))
        return;

    if (volume < 0.0f)
        volume = 0.0f;
    else if (volume > 1.0f)
        volume = 1.0f;

    const uint32_t q = (uint32_t)(volume * 1000000.0f + 0.5f);
    atomic_store_explicit(&target_volume_q, q, memory_order_release);
}

/*
 * Fade the final output to silence before changing active DSP settings.
 * The output task owns current_transition_gain; control tasks only publish
 * targets. If no stream is playing, no PCM is processed and the fade is a no-op.
 */
void i2s_output_transition_begin(void)
{
    if (!tx_handle)
        return;

    atomic_store_explicit(&target_transition_q, 0U, memory_order_release);
    vTaskDelay(pdMS_TO_TICKS(20));
}

void i2s_output_transition_end(void)
{
    atomic_store_explicit(&target_transition_q, 1000000U, memory_order_release);
}

/* Request a 10 ms fade-in on the first PCM block of a new AirPlay stream. */
void i2s_output_request_fade_in(void)
{
    atomic_store_explicit(&target_transition_q, 1000000U, memory_order_release);
    atomic_store_explicit(&transition_restart_requested, true, memory_order_release);
}

void i2s_output_init(int bck_pin, int ws_pin, int dout_pin)
{
    esp_err_t ret;

    current_volume = 0.05f;
    current_transition_gain = 1.0f;
    atomic_store_explicit(&target_volume_q, 50000U, memory_order_relaxed);
    atomic_store_explicit(&target_transition_q, 1000000U, memory_order_relaxed);
    atomic_store_explicit(&transition_restart_requested, false, memory_order_relaxed);

    ESP_LOGI(TAG, "Initializing I2S with pins - BCK: %d, WS: %d, DOUT: %d",
             bck_pin, ws_pin, dout_pin);

    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;

    ret = i2s_new_channel(&chan_cfg, &tx_handle, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 44100,
            .clk_src = I2S_CLK_SRC_XTAL,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bck_pin,
            .ws = ws_pin,
            .dout = dout_pin,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(tx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S standard mode: %s",
                 esp_err_to_name(ret));
        i2s_del_channel(tx_handle);
        tx_handle = NULL;
        return;
    }

    ret = i2s_channel_enable(tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));
        i2s_del_channel(tx_handle);
        tx_handle = NULL;
        return;
    }

    ESP_LOGI(TAG, "I2S initialized successfully; final volume stage enabled");
}

void i2s_output_write(const uint8_t *data, size_t len)
{
    if (!tx_handle) {
        ESP_LOGW(TAG, "I2S not initialized");
        return;
    }
    if (!data || len == 0 || (len & 0x03U) != 0) {
        ESP_LOGW(TAG, "Invalid I2S PCM buffer: %u bytes", (unsigned)len);
        return;
    }

    const int16_t *samples = (const int16_t *)data;
    const size_t frame_count = len / (2U * sizeof(int16_t));
    const float target_volume = (float)atomic_load_explicit(
        &target_volume_q, memory_order_acquire) * 0.000001f;
    const float target_transition = (float)atomic_load_explicit(
        &target_transition_q, memory_order_acquire) * 0.000001f;

    /* Exact unity bypass: no copy, no dither, no extra arithmetic. */
    if (target_volume >= 0.999999f && current_volume >= 0.999999f &&
        target_transition >= 0.999999f && current_transition_gain >= 0.999999f &&
        !atomic_load_explicit(&transition_restart_requested, memory_order_acquire)) {
        current_volume = 1.0f;
        size_t bytes_written = 0;
        const esp_err_t ret = i2s_channel_write(
            tx_handle, data, len, &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(ret));
        } else if (bytes_written != len) {
            ESP_LOGW(TAG, "I2S partial write: %u/%u bytes",
                     (unsigned)bytes_written, (unsigned)len);
        }
        return;
    }

    if (atomic_exchange_explicit(
            &transition_restart_requested, false, memory_order_acq_rel)) {
        current_transition_gain = 0.0f;
    }

    size_t frame_offset = 0;
    const size_t chunk_frames_max = I2S_VOLUME_BUFFER_SAMPLES / 2U;
    while (frame_offset < frame_count) {
        size_t chunk_frames = frame_count - frame_offset;
        if (chunk_frames > chunk_frames_max)
            chunk_frames = chunk_frames_max;

        for (size_t frame = 0; frame < chunk_frames; ++frame) {
            /* Update AirPlay and transition gains ONCE per stereo frame. */
            current_volume += I2S_VOLUME_RAMP_ALPHA *
                (target_volume - current_volume);
            if (target_volume <= 0.0f && current_volume < 0.0001f)
                current_volume = 0.0f;

            if (current_transition_gain < target_transition) {
                current_transition_gain += 1.0f / (float)I2S_TRANSITION_RAMP_SAMPLES;
                if (current_transition_gain > target_transition)
                    current_transition_gain = target_transition;
            } else if (current_transition_gain > target_transition) {
                current_transition_gain -= 1.0f / (float)I2S_TRANSITION_RAMP_SAMPLES;
                if (current_transition_gain < target_transition)
                    current_transition_gain = target_transition;
            }

            const float output_gain = current_volume * current_transition_gain;
            const size_t sample = (frame_offset + frame) * 2U;
            volume_buffer[frame * 2U] =
                scale_and_quantize(samples[sample], output_gain);
            volume_buffer[frame * 2U + 1U] =
                scale_and_quantize(samples[sample + 1U], output_gain);
        }

        if (fabsf(target_volume - current_volume) < 1.0e-7f)
            current_volume = target_volume;

        const size_t chunk_samples = chunk_frames * 2U;
        const size_t chunk_bytes = chunk_samples * sizeof(int16_t);
        size_t bytes_written = 0;
        const esp_err_t ret = i2s_channel_write(
            tx_handle, volume_buffer, chunk_bytes,
            &bytes_written, portMAX_DELAY);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(ret));
            return;
        }
        if (bytes_written != chunk_bytes) {
            ESP_LOGW(TAG, "I2S partial write: %u/%u bytes",
                     (unsigned)bytes_written, (unsigned)chunk_bytes);
            return;
        }

        frame_offset += chunk_frames;
    }
}

void i2s_output_deinit(void)
{
    if (tx_handle) {
        i2s_channel_disable(tx_handle);
        i2s_del_channel(tx_handle);
        tx_handle = NULL;
        ESP_LOGI(TAG, "I2S deinitialized");
    }
}
