#include "spdif_in.h"
#include "histogram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "string.h"
#include "math.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <stdatomic.h>
#include "freertos/semphr.h"

// Global state
static rmt_channel_handle_t g_rx_channel = NULL;
static TaskHandle_t g_decoder_task = NULL;
static SemaphoreHandle_t g_decoder_done = NULL;
static volatile bool g_running = false;
static _Atomic uint32_t g_last_pcm_ms = 0;
static _Atomic uint32_t g_decoded_frames = 0;
static _Atomic uint32_t g_pcm_drops = 0;
static _Atomic uint32_t g_sample_rate_hz = 0;
static rmt_symbol_word_t *g_rmt_buffer = NULL;
static _Atomic size_t g_rx_symbols_count = 0;
static rmt_receive_config_t g_rx_config;

// 256-byte LUT for pulse classification
static uint8_t pulse_lut[256];

RingbufHandle_t spdif_in_pcm_buffer = NULL;

static void spdif_emit_pcm_frame(int16_t left, int16_t right)
{
    const int16_t frame[2] = { left, right };
    atomic_store_explicit(&g_last_pcm_ms,
                          (uint32_t)(esp_timer_get_time() / 1000),
                          memory_order_release);
    atomic_fetch_add_explicit(&g_decoded_frames, 1U, memory_order_relaxed);

    if (!spdif_in_pcm_buffer ||
        xRingbufferSend(spdif_in_pcm_buffer, frame, sizeof(frame), 0) != pdTRUE) {
        atomic_fetch_add_explicit(&g_pcm_drops, 1U, memory_order_relaxed);
    }
}

static uint32_t spdif_rate_from_timing(void)
{
    if (g_timing.base_unit_ticks == 0) return 0;

    // One full BMC cell spans two shortest pulses and contains 1/(64*Fs).
    // Estimate Fs from the configured RMT tick resolution, then snap only
    // to the supported consumer rates when the estimate is within 3%.
    const float measured_hz =
        (float)RMT_RESOLUTION_HZ /
        ((float)g_timing.base_unit_ticks * 64.0f);
    const float error_44100 = fabsf(measured_hz - 44100.0f) / 44100.0f;
    const float error_48000 = fabsf(measured_hz - 48000.0f) / 48000.0f;

    if (error_44100 < error_48000 && error_44100 < 0.03f) return 44100;
    if (error_48000 < 0.03f) return 48000;
    return 0;
}

#define TIMING_VARIANCE 3

// Preamble patterns - both normal and inverted
#define PREAMBLE_B_0 0xE8
#define PREAMBLE_B_1 0x17
#define PREAMBLE_M_0 0xE2
#define PREAMBLE_M_1 0x1D
#define PREAMBLE_W_0 0xE4
#define PREAMBLE_W_1 0x1B

// Macro to process each duration - uses LUT for pulse classification
#define PROCESS_SYMBOL(dur)                                                          \
    {                                                                                \
        uint32_t ptype = pulse_lut[dur & 0xFF];                                      \
        if (ptype < 3)                                                               \
        {                                                                            \
                                                                                     \
            if (state & 2)                                                           \
            {               /* in_preamble */                                        \
                state ^= 8; /* Toggle level */                                       \
                                                                                     \
                uint32_t bits_to_add = ptype + 1;                                    \
                uint32_t pindex = (preamble_data >> 8) & 0xF;                        \
                uint32_t pattern = preamble_data & 0xFF;                             \
                                                                                     \
                for (uint32_t j = 0; j < bits_to_add && pindex < 8; j++)             \
                {                                                                    \
                    if (state & 8)                                                   \
                        pattern |= (1 << (7 - pindex));                              \
                    pindex++;                                                        \
                }                                                                    \
                                                                                     \
                if (pindex >= 8)                                                     \
                {                                                                    \
                    state &= ~2; /* Clear in_preamble */                             \
                                                                                     \
                    /* Check all 6 valid preambles */                                \
                    if (pattern == PREAMBLE_B_0 || pattern == PREAMBLE_B_1)          \
                    {                                                                \
                        channel = 0;                                                 \
                    }                                                                \
                    else if (pattern == PREAMBLE_M_0 || pattern == PREAMBLE_M_1)     \
                    {                                                                \
                        channel = 0;                                                 \
                    }                                                                \
                    else if (pattern == PREAMBLE_W_0 || pattern == PREAMBLE_W_1)     \
                    {                                                                \
                        channel = 1;                                                 \
                    }                                                                \
                }                                                                    \
                else                                                                 \
                {                                                                    \
                    preamble_data = pattern | (pindex << 8);                         \
                }                                                                    \
            }                                                                        \
            else if (ptype == 2 && !(state & 1))                                     \
            { /* LONG pulse, not expecting_short */                                  \
                /* Start preamble */                                                 \
                state |= 2; /* Set in_preamble */                                    \
                preamble_data = 0;                                                   \
                                                                                     \
                /* Set initial level based on last_data_bit and toggle */            \
                state = (state & ~8) | ((state & 4) << 1); /* Copy bit 2 to bit 3 */ \
                state ^= 8;                                /* Toggle */              \
                                                                                     \
                /* Add 3 bits for LONG */                                            \
                uint32_t pattern = 0;                                                \
                if (state & 8)                                                       \
                    pattern = 0xE0; /* 111 in top 3 bits */                          \
                preamble_data = pattern | (3 << 8);                                  \
                                                                                     \
                /* Reset subframe */                                                 \
                bit_count = 0;                                                       \
                subframe_data = 0;                                                   \
                state &= ~1; /* Clear expecting_short */                             \
            }                                                                        \
            else if (bit_count < 28)                                                 \
            { /* Normal data */                                                      \
                if (state & 1)                                                       \
                { /* expecting_short */                                              \
                    if (ptype == 0)                                                  \
                    { /* SHORT - completes '1' bit */                                \
                        subframe_data |= (1UL << bit_count);                         \
                    }                                                                \
                    bit_count++;                                                     \
                    state &= ~1; /* Clear expecting_short */                         \
                }                                                                    \
                else                                                                 \
                {                                                                    \
                    if (ptype == 1)                                                  \
                    { /* MEDIUM - '0' bit */                                         \
                        bit_count++;                                                 \
                    }                                                                \
                    else if (ptype == 0)                                             \
                    {               /* SHORT - first half of '1' */                  \
                        state |= 1; /* Set expecting_short */                        \
                    }                                                                \
                }                                                                    \
                                                                                     \
                if (bit_count == 28)                                                 \
                {                                                                    \
                    /* Track last bit for next preamble */                           \
                    state = (state & ~4) | ((subframe_data & (1UL << 27)) ? 4 : 0);  \
                                                                                     \
                    /* Extract audio */                                              \
                    int32_t sample = (int32_t)(subframe_data & 0xFFFFFF);            \
                                                                                     \
                    /* Sign extend from 24-bit to 32-bit */                          \
                    if (sample & 0x800000)                                           \
                    {                                                                \
                        sample |= 0xFF000000;                                        \
                    }                                                                \
                                                                                     \
                    /* Convert to 16-bit */                                          \
                    int16_t s16 = (int16_t)(sample >> 8);                            \
                                                                                     \
                    if (channel == 0)                                                \
                    {                                                                \
                        left_sample = s16;                                           \
                    }                                                                \
                    else                                                             \
                    {                                                                \
                        int16_t stereo[2] = {left_sample, s16};                      \
                        xRingbufferSend(spdif_in_pcm_buffer, stereo, sizeof(stereo), 10000);  \
                    }                                                                \
                }                                                                    \
            }                                                                        \
        }                                                                            \
    }

// Initialize thresholds and build LUT
void decoder_init_thresholds(void)
{
    for (int i = 0; i < 256; i++)
    {
        if (i < g_timing.short_pulse_ticks - TIMING_VARIANCE)
        {
            pulse_lut[i] = 3; // UNKNOWN
        }
        else if (i < g_timing.short_pulse_ticks + TIMING_VARIANCE)
        {
            pulse_lut[i] = 0; // SHORT
        }
        else if (i < g_timing.medium_pulse_ticks - TIMING_VARIANCE)
        {
            pulse_lut[i] = 3; // UNKNOWN
        }
        else if (i < g_timing.medium_pulse_ticks + TIMING_VARIANCE)
        {
            pulse_lut[i] = 1; // MEDIUM
        }
        else if (i < g_timing.long_pulse_ticks - TIMING_VARIANCE)
        {
            pulse_lut[i] = 3; // UNKNOWN
        }
        else if (i < g_timing.long_pulse_ticks + TIMING_VARIANCE)
        {
            pulse_lut[i] = 2; // LONG
        }
        else
        {
            pulse_lut[i] = 3; // UNKNOWN
        }
    }
}

// Main decoder task. IDF 5.2 has finite RMT RX transactions; re-arm in task
// context after each DMA capture rather than using the newer partial-RX flag.
static void spdif_decoder_task(void *arg)
{
    (void)arg;
    g_rx_config = (rmt_receive_config_t) {
        .signal_range_min_ns = 10,
        .signal_range_max_ns = 10000,
    };

    esp_err_t err = rmt_enable(g_rx_channel);
    if (err == ESP_OK) {
        err = rmt_receive(g_rx_channel, g_rmt_buffer,
                          RMT_MEM_BLOCK_SYMBOLS * sizeof(rmt_symbol_word_t),
                          &g_rx_config);
    }
    if (err != ESP_OK) {
        ESP_LOGE("SPDIF_IN", "Initial RMT receive failed: %s", esp_err_to_name(err));
        goto task_exit;
    }

    uint32_t state = 0;
    uint32_t bit_count = 0;
    uint32_t subframe_data = 0;
    uint32_t preamble_data = 0;
    uint32_t channel = 0;
    int16_t left_sample = 0;
    int64_t last_batch_us = 0;

    ESP_LOGI("SPDIF_IN", "IDF 5.2 DMA decoder started");
    while (g_running) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!g_running) break;

        const size_t num_symbols = atomic_exchange_explicit(
            &g_rx_symbols_count, 0, memory_order_acq_rel);
        if (num_symbols > 0 && g_rmt_buffer) {
            rmt_symbol_word_t *symbols = g_rmt_buffer;
            const int64_t now_us = esp_timer_get_time();

            // A long loss of carrier starts a fresh acquisition window. This
            // also allows a 44.1 <-> 48 kHz source change to be re-detected.
            if (last_batch_us != 0 && (now_us - last_batch_us) > 250000) {
                memset(&g_timing, 0, sizeof(g_timing));
                memset(pulse_lut, 0, sizeof(pulse_lut));
                atomic_store_explicit(&g_sample_rate_hz, 0, memory_order_release);
                atomic_store_explicit(&g_decoded_frames, 0, memory_order_relaxed);
                atomic_store_explicit(&g_last_pcm_ms, 0, memory_order_release);
                state = bit_count = subframe_data = preamble_data = channel = 0;
                left_sample = 0;
                ESP_LOGI("SPDIF_IN", "Carrier gap >250 ms: reacquiring S/PDIF timing");
            }
            last_batch_us = now_us;

            if (!g_timing.timing_discovered) {
                collect_pulse_histogram(symbols, num_symbols);
                if (g_timing.total_samples >= MIN_SAMPLES_FOR_ANALYSIS) {
                    analyze_pulse_timing();
                    if (g_timing.timing_discovered) {
                        const uint32_t rate = spdif_rate_from_timing();
                        atomic_store_explicit(&g_sample_rate_hz, rate, memory_order_release);
                        ESP_LOGI("SPDIF_IN", "S/PDIF timing acquired: %lu Hz",
                                 (unsigned long)rate);
                    }
                }
            } else {
                if (!pulse_lut[0]) decoder_init_thresholds();
                for (size_t i = 0; i < num_symbols; ++i) {
                    PROCESS_SYMBOL(symbols[i].duration0);
                    PROCESS_SYMBOL(symbols[i].duration1);
                }
            }

        }

        // An absent optical carrier can complete each RX transaction by timeout.
        // Back off instead of spinning the CPU while Optical is unplugged/idle.
        if (num_symbols == 0 && g_running) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // On IDF 5.2 the RX transaction completes on buffer-full or signal
        // timeout. Re-arm from task context; the BMC preamble re-synchronizes
        // the decoder after the short gap between transactions.
        if (g_running) {
            do {
                err = rmt_receive(g_rx_channel, g_rmt_buffer,
                                  RMT_MEM_BLOCK_SYMBOLS * sizeof(rmt_symbol_word_t),
                                  &g_rx_config);
                if (err != ESP_OK) {
                    ESP_LOGW("SPDIF_IN", "RMT re-arm failed: %s", esp_err_to_name(err));
                    vTaskDelay(pdMS_TO_TICKS(2));
                }
            } while (g_running && err != ESP_OK);
        }
    }

task_exit:
    if (g_decoder_done) xSemaphoreGive(g_decoder_done);
    g_decoder_task = NULL;
    vTaskDelete(NULL);
}

// RMT receive callback (ISR context): only ISR-safe FreeRTOS calls here.
static bool IRAM_ATTR rmt_rx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *edata,
    void *user_ctx)
{
    (void)channel;
    (void)user_ctx;
    BaseType_t task_woken = pdFALSE;

    if (edata && edata->num_symbols > 0) {
        // Do not copy a full DMA block inside the ISR. The decoder task owns
        // this completed buffer until it re-arms the next RMT transaction.
        atomic_store_explicit(&g_rx_symbols_count, edata->num_symbols,
                              memory_order_release);
    }

    if (g_decoder_task) {
        vTaskNotifyGiveFromISR(g_decoder_task, &task_woken);
    }
    return task_woken == pdTRUE;
}

// Initialize S/PDIF receiver.
esp_err_t spdif_receiver_init(int input_pin, void (*init_done_cb)(void))
{
    if (g_rx_channel || g_decoder_task) return ESP_ERR_INVALID_STATE;
    ESP_LOGI("SPDIF_IN", "Initializing Optical RX on GPIO%d", input_pin);

    atomic_store_explicit(&g_last_pcm_ms, 0, memory_order_relaxed);
    atomic_store_explicit(&g_decoded_frames, 0, memory_order_relaxed);
    atomic_store_explicit(&g_pcm_drops, 0, memory_order_relaxed);
    atomic_store_explicit(&g_sample_rate_hz, 0, memory_order_relaxed);

    spdif_in_pcm_buffer = xRingbufferCreate(SPDIF_PCM_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!spdif_in_pcm_buffer) return ESP_ERR_NO_MEM;

    g_rmt_buffer = heap_caps_malloc(
        RMT_MEM_BLOCK_SYMBOLS * sizeof(rmt_symbol_word_t),
        MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!g_rmt_buffer) {
        vRingbufferDelete(spdif_in_pcm_buffer);
        spdif_in_pcm_buffer = NULL;
        return ESP_ERR_NO_MEM;
    }

    const rmt_rx_channel_config_t rx_channel_cfg = {
        .gpio_num = input_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = RMT_MEM_BLOCK_SYMBOLS,
        .flags.with_dma = true,
    };

    esp_err_t ret = rmt_new_rx_channel(&rx_channel_cfg, &g_rx_channel);
    if (ret != ESP_OK) {
        ESP_LOGE("SPDIF_IN", "RMT RX allocation failed: %s", esp_err_to_name(ret));
        spdif_receiver_deinit();
        return ret;
    }

    const rmt_rx_event_callbacks_t cbs = { .on_recv_done = rmt_rx_done_callback };
    ret = rmt_rx_register_event_callbacks(g_rx_channel, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE("SPDIF_IN", "RMT callback registration failed: %s", esp_err_to_name(ret));
        spdif_receiver_deinit();
        return ret;
    }

    g_decoder_done = xSemaphoreCreateBinary();
    if (!g_decoder_done) {
        spdif_receiver_deinit();
        return ESP_ERR_NO_MEM;
    }

    if (init_done_cb) init_done_cb();

    g_running = true;
    if (xTaskCreatePinnedToCore(
            spdif_decoder_task, "spdif_decoder",
            DECODER_TASK_STACK, NULL, DECODER_TASK_PRIORITY,
            &g_decoder_task, 1) != pdPASS) {
        g_running = false;
        spdif_receiver_deinit();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI("SPDIF_IN", "RMT DMA capture configured: %u symbols @ %lu Hz",
             (unsigned)RMT_MEM_BLOCK_SYMBOLS, (unsigned long)RMT_RESOLUTION_HZ);
    return ESP_OK;
}

esp_err_t spdif_receiver_start(void)
{
    return g_decoder_task ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t spdif_receiver_stop(void)
{
    g_running = false;
    if (g_rx_channel) {
        const esp_err_t err = rmt_disable(g_rx_channel);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE("SPDIF_IN", "RMT disable failed: %s", esp_err_to_name(err));
            return err;
        }
    }
    if (g_decoder_task) vTaskNotifyGive(g_decoder_task);
    if (g_decoder_done && g_decoder_task) {
        if (xSemaphoreTake(g_decoder_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGE("SPDIF_IN", "Decoder task did not stop; keeping resources allocated");
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

void spdif_receiver_deinit(void)
{
    const esp_err_t stop_err = spdif_receiver_stop();
    if (stop_err != ESP_OK) {
        ESP_LOGE("SPDIF_IN", "Deinit aborted to avoid freeing active DMA resources");
        return;
    }

    if (g_rx_channel) {
        (void)rmt_del_channel(g_rx_channel);
        g_rx_channel = NULL;
    }
    if (g_rmt_buffer) {
        heap_caps_free(g_rmt_buffer);
        g_rmt_buffer = NULL;
    }
    if (spdif_in_pcm_buffer) {
        vRingbufferDelete(spdif_in_pcm_buffer);
        spdif_in_pcm_buffer = NULL;
    }
    if (g_decoder_done) {
        vSemaphoreDelete(g_decoder_done);
        g_decoder_done = NULL;
    }
    atomic_store_explicit(&g_sample_rate_hz, 0, memory_order_release);
    atomic_store_explicit(&g_last_pcm_ms, 0, memory_order_release);
}

uint32_t spdif_receiver_get_sample_rate(void)
{
    return atomic_load_explicit(&g_sample_rate_hz, memory_order_acquire);
}

bool spdif_receiver_is_locked(void)
{
    const uint32_t sample_rate =
        atomic_load_explicit(&g_sample_rate_hz, memory_order_acquire);
    const uint32_t frames =
        atomic_load_explicit(&g_decoded_frames, memory_order_relaxed);
    const uint32_t last_pcm_ms =
        atomic_load_explicit(&g_last_pcm_ms, memory_order_acquire);
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if ((sample_rate != 44100 && sample_rate != 48000) ||
        frames < 1024 || last_pcm_ms == 0) {
        return false;
    }
    return (uint32_t)(now_ms - last_pcm_ms) < 150U;
}

uint32_t spdif_receiver_get_decoded_frames(void)
{
    return atomic_load_explicit(&g_decoded_frames, memory_order_relaxed);
}

uint32_t spdif_receiver_get_pcm_drops(void)
{
    return atomic_load_explicit(&g_pcm_drops, memory_order_relaxed);
}

