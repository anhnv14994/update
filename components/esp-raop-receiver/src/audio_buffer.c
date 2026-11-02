#include "audio_buffer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdatomic.h>

static const char *TAG = "audio_buffer";

/* ========================================================================
 * Configuration
 * ======================================================================== */

#define BUFFER_FRAMES             384
#define MAX_FRAME_SIZE            2048
#define SAMPLE_RATE_HZ            44100

#define I2S_LEAD_US               (40 * 1000LL)
#define ALIGN_SLACK_US            (150 * 1000LL)
#define UNDERRUN_STARTUP_GRACE_MS 750
#define MAX_PAD_US                (40 * 1000LL)
#define SILENCE_CHUNK_FRAMES      352

#define DRIFT_DEADBAND_US         (10 * 1000LL)
#define DRIFT_MIN_INTERVAL_MS     125
#define DRIFT_EMA_SHIFT           6
#define DRIFT_COMPENSATION_ENABLED 1

#define DSP_PIPE_FRAMES           16
#define DSP_TASK_STACK_SIZE       8192
/* Keep realtime I2S output above the background DSP worker. */
#define DSP_TASK_PRIORITY         2
#define DSP_TASK_CORE             0

#define AUDIO_TASK_STACK_SIZE     12288
#define AUDIO_TASK_PRIORITY       3
#define AUDIO_TASK_CORE           1

#define AUDIO_STOP_WAIT_MS        500

#define SAMPLE_BYTES              4

/* ========================================================================
 * SPSC frame types
 *
 * Ownership is defined by indices, not by per-slot mutex/ready flags.
 * ======================================================================== */

typedef struct {
    uint8_t  data[MAX_FRAME_SIZE];
    size_t   len;
    uint32_t playtime;
} audio_frame_t;

typedef struct {
    uint8_t  data[MAX_FRAME_SIZE];
    size_t   len;
    uint32_t playtime;
} dsp_frame_t;

/* ========================================================================
 * Stats
 *
 * Multiple tasks update some counters, so use atomics.
 * ======================================================================== */

typedef struct {
    _Atomic uint32_t underruns;
    _Atomic uint32_t pads;
    _Atomic uint32_t drops;
    _Atomic uint32_t overruns;
    _Atomic uint32_t tap_calls;
    _Atomic uint32_t dsp_waits;
    _Atomic uint32_t dsp_drops;
    _Atomic uint32_t dsp_total_us;
    _Atomic uint32_t dsp_max_us;
    _Atomic uint32_t dsp_total_samples;
    uint32_t last_log_ms;
} audio_stats_t;

/* ========================================================================
 * Audio state
 * ======================================================================== */

static struct {
    audio_frame_t *frames;

    /* Raw SPSC ring: RTP producer -> DSP consumer (or output consumer). */
    _Atomic uint32_t read_idx;
    _Atomic uint32_t write_idx;

    /* Task lifecycle. */
    SemaphoreHandle_t task_done;
    TaskHandle_t task;

    SemaphoreHandle_t dsp_done;
    TaskHandle_t dsp_task;

    volatile bool running;
    volatile uint32_t start_time;

    audio_output_callback_t output_cb;
    void *user_ctx;

    audio_pcm_tap_cb_t tap_cb;
    void *tap_user_ctx;
} audio_buf = {0};

/* ========================================================================
 * DSP SPSC ring: DSP producer -> output consumer
 * ======================================================================== */

static dsp_frame_t *dsp_frames = NULL;

static _Atomic uint32_t dsp_read_idx = 0;
static _Atomic uint32_t dsp_write_idx = 0;

/* ========================================================================
 * Control requests
 *
 * flush_requested is written by the RAOP control path and consumed by
 * audio_output_task. flush_generation is owned by the output-side flush
 * coordinator and observed by audio_dsp_task.
 * ======================================================================== */

static _Atomic bool flush_requested = false;
static _Atomic uint32_t flush_generation = 0;

static struct {
    _Atomic uint32_t skip_frames;
    _Atomic uint32_t pause_frames;
} timing_correction = {0};

/* ========================================================================
 * DAC timeline
 * ======================================================================== */

static int64_t dac_end_us = 0;
static int64_t underrun_grace_until_us = 0;
static audio_stats_t out_stats = {0};

/* ========================================================================
 * Drift state (output task only)
 * ======================================================================== */

static int64_t  drift_ema_us = 0;
static bool     drift_ema_init = false;
static uint32_t drift_last_adjust_ms = 0;
static uint32_t drift_inserts = 0;
static uint32_t drift_drops = 0;

/* ========================================================================
 * Static work buffers
 * ======================================================================== */

static uint8_t s_output_copy[MAX_FRAME_SIZE];
static uint8_t s_silence[SILENCE_CHUNK_FRAMES * 4];

/* ========================================================================
 * Ring helpers
 * ======================================================================== */

static inline uint32_t ring_next(uint32_t idx)
{
    return (idx + 1U) % BUFFER_FRAMES;
}

static inline uint32_t dsp_ring_next(uint32_t idx)
{
    return (idx + 1U) % DSP_PIPE_FRAMES;
}

static inline bool raw_ring_empty(uint32_t r, uint32_t w)
{
    return r == w;
}

static inline bool raw_ring_full(uint32_t r, uint32_t w)
{
    return ring_next(w) == r;
}

static inline bool dsp_ring_empty(uint32_t r, uint32_t w)
{
    return r == w;
}

static inline bool dsp_ring_full(uint32_t r, uint32_t w)
{
    return dsp_ring_next(w) == r;
}

static inline uint32_t raw_ring_count(uint32_t r, uint32_t w)
{
    if (w >= r)
        return w - r;

    return BUFFER_FRAMES - r + w;
}

static inline uint32_t dsp_ring_count(uint32_t r, uint32_t w)
{
    if (w >= r)
        return w - r;

    return DSP_PIPE_FRAMES - r + w;
}

/* ========================================================================
 * Time helpers
 * ======================================================================== */

static inline int64_t frame_us(size_t bytes)
{
    return ((int64_t)(bytes / 4) * 1000000LL) / SAMPLE_RATE_HZ;
}

static inline uint32_t gettime_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ========================================================================
 * Stats helpers
 * ======================================================================== */

static void stats_reset(void)
{
    atomic_store_explicit(&out_stats.underruns, 0, memory_order_relaxed);
    atomic_store_explicit(&out_stats.pads, 0, memory_order_relaxed);
    atomic_store_explicit(&out_stats.drops, 0, memory_order_relaxed);
    atomic_store_explicit(&out_stats.overruns, 0, memory_order_relaxed);
    atomic_store_explicit(&out_stats.tap_calls, 0, memory_order_relaxed);
    atomic_store_explicit(&out_stats.dsp_waits, 0, memory_order_relaxed);
    atomic_store_explicit(&out_stats.dsp_drops, 0, memory_order_relaxed);
    atomic_store_explicit(&out_stats.dsp_total_us, 0, memory_order_relaxed);
    atomic_store_explicit(&out_stats.dsp_max_us, 0, memory_order_relaxed);
    atomic_store_explicit(&out_stats.dsp_total_samples, 0, memory_order_relaxed);
    out_stats.last_log_ms = gettime_ms();
}

/* ========================================================================
 * Skip helper
 * ======================================================================== */

static bool consume_skip_one(void)
{
    uint32_t cur = atomic_load_explicit(
        &timing_correction.skip_frames,
        memory_order_relaxed);

    while (cur > 0) {
        if (atomic_compare_exchange_weak_explicit(
                &timing_correction.skip_frames,
                &cur,
                cur - 1U,
                memory_order_acq_rel,
                memory_order_relaxed)) {
            return true;
        }
    }

    return false;
}

/* ========================================================================
 * Drift
 * ======================================================================== */

static int drift_step(int64_t level_us, uint32_t now_ms)
{
    if (!drift_ema_init) {
        drift_ema_us = level_us;
        drift_ema_init = true;
    } else {
        drift_ema_us +=
            (level_us - drift_ema_us) >> DRIFT_EMA_SHIFT;
    }

    if ((uint32_t)(now_ms - drift_last_adjust_ms) <
        DRIFT_MIN_INTERVAL_MS) {
        return 0;
    }

    if (drift_ema_us < I2S_LEAD_US - DRIFT_DEADBAND_US) {
        drift_last_adjust_ms = now_ms;
        drift_inserts++;
        return +1;
    }

    if (drift_ema_us > I2S_LEAD_US + DRIFT_DEADBAND_US) {
        drift_last_adjust_ms = now_ms;
        drift_drops++;
        return -1;
    }

    return 0;
}

/* ========================================================================
 * Silence
 * ======================================================================== */

static void write_silence_us(int64_t us)
{
    while (us > 0 && audio_buf.running) {
        int64_t frames =
            (us * SAMPLE_RATE_HZ) / 1000000LL;

        if (frames <= 0)
            break;

        if (frames > SILENCE_CHUNK_FRAMES)
            frames = SILENCE_CHUNK_FRAMES;

        size_t bytes = (size_t)frames * 4;

        memset(s_silence, 0, bytes);

        if (audio_buf.output_cb) {
            audio_buf.output_cb(
                s_silence,
                bytes,
                audio_buf.user_ctx);
        }

        us -= frame_us(bytes);
    }
}

/* ========================================================================
 * Flush handling
 * ======================================================================== */

static void handle_flush_request(void)
{
    if (!atomic_exchange_explicit(
            &flush_requested,
            false,
            memory_order_acq_rel)) {
        return;
    }

    /* Invalidate any DSP work already in flight. */
    atomic_fetch_add_explicit(
        &flush_generation,
        1U,
        memory_order_acq_rel);

    /*
     * Output task owns the DSP read side.  Discard all already-published
     * processed frames by moving the consumer index to the producer index.
     */
    uint32_t dsp_w =
        atomic_load_explicit(
            &dsp_write_idx,
            memory_order_acquire);

    atomic_store_explicit(
        &dsp_read_idx,
        dsp_w,
        memory_order_release);

    /*
     * With DSP OFF, output task also owns the raw consumer side.
     * With DSP ON, DSP task will observe flush_generation and move raw_read
     * to raw_write itself.
     */
    if (!audio_buf.tap_cb) {
        uint32_t raw_w =
            atomic_load_explicit(
                &audio_buf.write_idx,
                memory_order_acquire);

        atomic_store_explicit(
            &audio_buf.read_idx,
            raw_w,
            memory_order_release);
    }

    atomic_store_explicit(
        &timing_correction.skip_frames,
        0,
        memory_order_relaxed);

    atomic_store_explicit(
        &timing_correction.pause_frames,
        0,
        memory_order_relaxed);

    dac_end_us = 0;
    underrun_grace_until_us = 0;
    audio_buf.start_time = 0;

    ESP_LOGD(TAG, "Lock-free buffer flushed");
}

/* ========================================================================
 * DSP task
 * ======================================================================== */

static void audio_dsp_task(void *arg)
{
    (void)arg;

    uint32_t seen_flush_generation =
        atomic_load_explicit(
            &flush_generation,
            memory_order_acquire);

    ESP_LOGI(
        TAG,
        "DSP task started (core=%d priority=%d)",
        DSP_TASK_CORE,
        DSP_TASK_PRIORITY);

    while (audio_buf.running) {

        /* -------------------------------------------------------------
         * Handle a flush coordinated by output_task.
         * ------------------------------------------------------------- */

        uint32_t current_generation =
            atomic_load_explicit(
                &flush_generation,
                memory_order_acquire);

        if (current_generation !=
            seen_flush_generation) {

            uint32_t raw_w =
                atomic_load_explicit(
                    &audio_buf.write_idx,
                    memory_order_acquire);

            atomic_store_explicit(
                &audio_buf.read_idx,
                raw_w,
                memory_order_release);

            seen_flush_generation =
                current_generation;

            continue;
        }

        /* -------------------------------------------------------------
         * Do not start another DSP frame if processed queue is full.
         * ------------------------------------------------------------- */

        uint32_t dsp_r =
            atomic_load_explicit(
                &dsp_read_idx,
                memory_order_acquire);

        uint32_t dsp_w =
            atomic_load_explicit(
                &dsp_write_idx,
                memory_order_relaxed);

        if (dsp_ring_full(dsp_r, dsp_w)) {
            atomic_fetch_add_explicit(
                &out_stats.dsp_waits,
                1,
                memory_order_relaxed);

            vTaskDelay(1);
            continue;
        }

        /* -------------------------------------------------------------
         * Get one raw frame.
         * ------------------------------------------------------------- */

        uint32_t raw_r =
            atomic_load_explicit(
                &audio_buf.read_idx,
                memory_order_relaxed);

        uint32_t raw_w =
            atomic_load_explicit(
                &audio_buf.write_idx,
                memory_order_acquire);

        if (raw_ring_empty(raw_r, raw_w)) {
            atomic_fetch_add_explicit(
                &out_stats.dsp_waits,
                1,
                memory_order_relaxed);

            vTaskDelay(1);
            continue;
        }

        uint32_t next_raw_r =
            ring_next(raw_r);

        /*
         * Preserve chronological skip order:
         * if there are already published DSP frames, output_task should
         * discard those first.  DSP may only directly discard a raw frame
         * when the processed queue is empty.
         */
        uint32_t skip_before =
            atomic_load_explicit(
                &timing_correction.skip_frames,
                memory_order_relaxed);

        if (skip_before > 0 &&
            dsp_ring_empty(dsp_r, dsp_w)) {

            if (consume_skip_one()) {
                atomic_store_explicit(
                    &audio_buf.read_idx,
                    next_raw_r,
                    memory_order_release);

                atomic_fetch_add_explicit(
                    &out_stats.drops,
                    1,
                    memory_order_relaxed);

                continue;
            }
        }

        /* -------------------------------------------------------------
         * Reserve current DSP output slot.
         * ------------------------------------------------------------- */

        const uint32_t processing_generation =
            current_generation;

        dsp_frame_t *dst =
            &dsp_frames[dsp_w];

        audio_frame_t *src =
            &audio_buf.frames[raw_r];

        size_t raw_len = src->len;

        if (raw_len > MAX_FRAME_SIZE)
            raw_len = MAX_FRAME_SIZE;

        /* -------------------------------------------------------------
         * RAW -> DSP copy.
         *
         * No mutex.  Producer cannot overwrite raw_r until read_idx is
         * advanced by this consumer.
         * ------------------------------------------------------------- */

        memcpy(
            dst->data,
            src->data,
            raw_len);

        dst->len = raw_len;
        dst->playtime = src->playtime;

        /* -------------------------------------------------------------
         * DSP processing outside both ring ownership domains.
         * ------------------------------------------------------------- */

        if (!audio_buf.running)
            break;

        if (audio_buf.tap_cb) {
            const int64_t dsp_start_us = esp_timer_get_time();

            audio_buf.tap_cb(
                dst->data,
                dst->len,
                audio_buf.tap_user_ctx);

            const uint32_t dsp_elapsed_us =
                (uint32_t)(esp_timer_get_time() - dsp_start_us);

            atomic_fetch_add_explicit(
                &out_stats.dsp_total_us,
                dsp_elapsed_us,
                memory_order_relaxed);

            atomic_fetch_add_explicit(
                &out_stats.dsp_total_samples,
                (uint32_t)(dst->len / SAMPLE_BYTES),
                memory_order_relaxed);

            uint32_t previous_max =
                atomic_load_explicit(
                    &out_stats.dsp_max_us,
                    memory_order_relaxed);

            while (dsp_elapsed_us > previous_max &&
                   !atomic_compare_exchange_weak_explicit(
                       &out_stats.dsp_max_us,
                       &previous_max,
                       dsp_elapsed_us,
                       memory_order_relaxed,
                       memory_order_relaxed)) {
            }

            atomic_fetch_add_explicit(
                &out_stats.tap_calls,
                1,
                memory_order_relaxed);
        }

        /* -------------------------------------------------------------
         * A flush invalidates the frame being processed.
         * ------------------------------------------------------------- */

        uint32_t generation_after_dsp =
            atomic_load_explicit(
                &flush_generation,
                memory_order_acquire);

        if (generation_after_dsp !=
            processing_generation) {
            continue;
        }

        /* -------------------------------------------------------------
         * If processed queue became empty while we worked and a skip is
         * still pending, this raw frame is now the earliest frame and can
         * be discarded directly.
         * ------------------------------------------------------------- */

        dsp_r =
            atomic_load_explicit(
                &dsp_read_idx,
                memory_order_acquire);

        dsp_w =
            atomic_load_explicit(
                &dsp_write_idx,
                memory_order_relaxed);

        if (atomic_load_explicit(
                &timing_correction.skip_frames,
                memory_order_relaxed) > 0 &&
            dsp_ring_empty(dsp_r, dsp_w)) {

            if (consume_skip_one()) {
                atomic_store_explicit(
                    &audio_buf.read_idx,
                    next_raw_r,
                    memory_order_release);

                atomic_fetch_add_explicit(
                    &out_stats.drops,
                    1,
                    memory_order_relaxed);

                continue;
            }
        }

        /* -------------------------------------------------------------
         * Consume raw frame first, then publish DSP frame.
         * If flush happens between them, generation check prevents the
         * old DSP frame from being published after the flush.
         * ------------------------------------------------------------- */

        atomic_store_explicit(
            &audio_buf.read_idx,
            next_raw_r,
            memory_order_release);

        if (atomic_load_explicit(
                &flush_generation,
                memory_order_acquire) !=
            processing_generation) {
            continue;
        }

        atomic_store_explicit(
            &dsp_write_idx,
            dsp_ring_next(dsp_w),
            memory_order_release);
    }

    ESP_LOGI(TAG, "DSP task exiting");

    if (audio_buf.dsp_done)
        xSemaphoreGive(audio_buf.dsp_done);

    vTaskDelete(NULL);
}

/* ========================================================================
 * Output task
 * ======================================================================== */

static void audio_output_task(void *arg)
{
    (void)arg;

    ESP_LOGD(
        TAG,
        "Audio output task started");

    while (audio_buf.running) {

        /* -------------------------------------------------------------
         * Flush is coordinated here because this task owns the DSP read
         * side.  This keeps both rings true SPSC on the hot path.
         * ------------------------------------------------------------- */

        handle_flush_request();

        if (audio_buf.start_time == 0) {
            ulTaskNotifyTake(
                pdTRUE,
                pdMS_TO_TICKS(10));
            continue;
        }

        /* -------------------------------------------------------------
         * Timing correction: skip earliest queued frames.
         * ------------------------------------------------------------- */

        uint32_t pending_skip =
            atomic_load_explicit(
                &timing_correction.skip_frames,
                memory_order_relaxed);

        if (pending_skip > 0 &&
            audio_buf.tap_cb) {

            uint32_t r =
                atomic_load_explicit(
                    &dsp_read_idx,
                    memory_order_relaxed);

            uint32_t w =
                atomic_load_explicit(
                    &dsp_write_idx,
                    memory_order_acquire);

            if (!dsp_ring_empty(r, w)) {

                if (atomic_load_explicit(
                        &flush_requested,
                        memory_order_acquire)) {
                    continue;
                }

                atomic_store_explicit(
                    &dsp_read_idx,
                    dsp_ring_next(r),
                    memory_order_release);

                if (consume_skip_one()) {
                    atomic_fetch_add_explicit(
                        &out_stats.drops,
                        1,
                        memory_order_relaxed);
                }

                continue;
            }
        }

        /* -------------------------------------------------------------
         * Timing correction: silence.
         * ------------------------------------------------------------- */

        uint32_t pause_frames =
            atomic_load_explicit(
                &timing_correction.pause_frames,
                memory_order_relaxed);

        if (pause_frames > 0) {

            if (atomic_load_explicit(
                    &flush_requested,
                    memory_order_acquire)) {
                continue;
            }

            memset(
                s_silence,
                0,
                sizeof(s_silence));

            int64_t now_us =
                esp_timer_get_time();

            int64_t start_us =
                (dac_end_us > now_us)
                    ? dac_end_us
                    : now_us;

            if (audio_buf.output_cb) {
                audio_buf.output_cb(
                    s_silence,
                    SILENCE_CHUNK_FRAMES * 4,
                    audio_buf.user_ctx);
            }

            dac_end_us =
                start_us +
                frame_us(
                    SILENCE_CHUNK_FRAMES * 4);

            atomic_fetch_sub_explicit(
                &timing_correction.pause_frames,
                1,
                memory_order_relaxed);

            continue;
        }

        /* -------------------------------------------------------------
         * Per-frame state.
         * ------------------------------------------------------------- */

        bool have_frame = false;
        bool drop = false;
        bool pad = false;

        size_t out_len = 0;
        uint32_t playtime = 0;

        int64_t pad_us = 0;
        int64_t now_us = 0;
        int64_t start_us = 0;

        /* =============================================================
         * DSP ON
         * ============================================================= */

        if (audio_buf.tap_cb) {

            uint32_t candidate_idx =
                atomic_load_explicit(
                    &dsp_read_idx,
                    memory_order_relaxed);

            uint32_t dsp_w =
                atomic_load_explicit(
                    &dsp_write_idx,
                    memory_order_acquire);

            if (dsp_ring_empty(candidate_idx, dsp_w)) {
                atomic_fetch_add_explicit(
                    &out_stats.dsp_waits,
                    1,
                    memory_order_relaxed);

                vTaskDelay(1);
                continue;
            }

            const uint32_t candidate_generation =
                atomic_load_explicit(
                    &flush_generation,
                    memory_order_acquire);

            dsp_frame_t *frame =
                &dsp_frames[candidate_idx];

            playtime =
                frame->playtime;

            out_len =
                frame->len;

            if (out_len > MAX_FRAME_SIZE)
                out_len = MAX_FRAME_SIZE;

            /* ---------------------------------------------------------
             * Timing calculation outside any ring lock.
             * --------------------------------------------------------- */

            now_us =
                esp_timer_get_time();

            uint32_t now_ms =
                (uint32_t)(now_us / 1000ULL);

            int32_t to_play_ms =
                (int32_t)(playtime - now_ms);

            int64_t rel_us =
                (int64_t)to_play_ms *
                1000LL -
                (now_us % 1000LL);

            /*
             * Wait against the later of:
             *   1) the current DAC timeline, and
             *   2) this frame's sender playtime.
             *
             * The old code only considered dac_end_us when it was ahead,
             * so a future frame could trigger a large PAD even though the
             * frame should simply have been held until its playout point.
             */
            int64_t schedule_us = rel_us;

            if (dac_end_us > now_us) {
                int64_t dac_rel_us =
                    dac_end_us - now_us;

                if (dac_rel_us > schedule_us)
                    schedule_us = dac_rel_us;
            }

            int64_t hold_us = 0;

            if (schedule_us > I2S_LEAD_US) {
                hold_us =
                    schedule_us -
                    I2S_LEAD_US;
            }

            if (hold_us > 0) {
                uint32_t wait_ms =
                    (uint32_t)(hold_us / 1000LL);

                if (wait_ms < 1)
                    wait_ms = 1;

                if (wait_ms > 10)
                    wait_ms = 10;

                vTaskDelay(pdMS_TO_TICKS(wait_ms));
                continue;
            }

            int64_t play_abs_us =
                now_us + rel_us;

            start_us =
                (dac_end_us > now_us)
                    ? dac_end_us
                    : now_us;

            int64_t err_us =
                start_us - play_abs_us;

            if (err_us < -ALIGN_SLACK_US) {
                /*
                 * Extremely future-dated frame: this should be rare after
                 * the max(dac_end, playtime) wait above.  Cap the silence
                 * insertion so the local DAC timeline cannot jump hundreds
                 * of milliseconds.
                 */
                pad_us = -err_us;

                if (pad_us > MAX_PAD_US)
                    pad_us = MAX_PAD_US;

                pad = pad_us > 0;

            } else if (err_us > ALIGN_SLACK_US) {
                /*
                 * Frame playtime is already far behind the local DAC.
                 * Keep the PCM stream continuous and follow the current
                 * local timeline instead of dropping a burst of valid audio.
                 */
                drop = false;
            }

            /* ---------------------------------------------------------
             * Flush requested while we were timing the frame.
             * Drop it instead of sending stale PCM to I2S.
             * --------------------------------------------------------- */

            if (atomic_load_explicit(
                    &flush_requested,
                    memory_order_acquire)) {
                continue;
            }

            /* ---------------------------------------------------------
             * Consume current DSP frame.
             * SPSC means no re-validation mutex is necessary.
             * --------------------------------------------------------- */

            if (drop) {
                atomic_store_explicit(
                    &dsp_read_idx,
                    dsp_ring_next(candidate_idx),
                    memory_order_release);

                atomic_fetch_add_explicit(
                    &out_stats.drops,
                    1,
                    memory_order_relaxed);

            } else {
                memcpy(
                    s_output_copy,
                    frame->data,
                    out_len);

                atomic_store_explicit(
                    &dsp_read_idx,
                    dsp_ring_next(candidate_idx),
                    memory_order_release);

                have_frame = true;
            }

            /* If a flush was coordinated while copying, do not output it. */
            if (candidate_generation !=
                atomic_load_explicit(
                    &flush_generation,
                    memory_order_acquire)) {
                have_frame = false;
                continue;
            }

        /* =============================================================
         * DSP OFF
         * ============================================================= */

        } else {

            uint32_t candidate_idx =
                atomic_load_explicit(
                    &audio_buf.read_idx,
                    memory_order_relaxed);

            uint32_t raw_w =
                atomic_load_explicit(
                    &audio_buf.write_idx,
                    memory_order_acquire);

            if (raw_ring_empty(candidate_idx, raw_w)) {
                vTaskDelay(1);
                continue;
            }

            const uint32_t candidate_generation =
                atomic_load_explicit(
                    &flush_generation,
                    memory_order_acquire);

            audio_frame_t *frame =
                &audio_buf.frames[candidate_idx];

            playtime = frame->playtime;
            out_len = frame->len;

            if (out_len > MAX_FRAME_SIZE)
                out_len = MAX_FRAME_SIZE;

            /* ---------------------------------------------------------
             * Timing outside the raw ring.
             * --------------------------------------------------------- */

            now_us =
                esp_timer_get_time();

            uint32_t now_ms =
                (uint32_t)(now_us / 1000ULL);

            int32_t to_play_ms =
                (int32_t)(playtime - now_ms);

            int64_t rel_us =
                (int64_t)to_play_ms *
                1000LL -
                (now_us % 1000LL);

            /*
             * Wait against the later of:
             *   1) the current DAC timeline, and
             *   2) this frame's sender playtime.
             *
             * The old code only considered dac_end_us when it was ahead,
             * so a future frame could trigger a large PAD even though the
             * frame should simply have been held until its playout point.
             */
            int64_t schedule_us = rel_us;

            if (dac_end_us > now_us) {
                int64_t dac_rel_us =
                    dac_end_us - now_us;

                if (dac_rel_us > schedule_us)
                    schedule_us = dac_rel_us;
            }

            int64_t hold_us = 0;

            if (schedule_us > I2S_LEAD_US) {
                hold_us =
                    schedule_us -
                    I2S_LEAD_US;
            }

            if (hold_us > 0) {
                uint32_t wait_ms =
                    (uint32_t)(hold_us / 1000LL);

                if (wait_ms < 1)
                    wait_ms = 1;

                if (wait_ms > 10)
                    wait_ms = 10;

                vTaskDelay(pdMS_TO_TICKS(wait_ms));
                continue;
            }

            int64_t play_abs_us =
                now_us + rel_us;

            start_us =
                (dac_end_us > now_us)
                    ? dac_end_us
                    : now_us;

            int64_t err_us =
                start_us - play_abs_us;

            if (err_us < -ALIGN_SLACK_US) {
                /*
                 * Extremely future-dated frame: this should be rare after
                 * the max(dac_end, playtime) wait above.  Cap the silence
                 * insertion so the local DAC timeline cannot jump hundreds
                 * of milliseconds.
                 */
                pad_us = -err_us;

                if (pad_us > MAX_PAD_US)
                    pad_us = MAX_PAD_US;

                pad = pad_us > 0;

            } else if (err_us > ALIGN_SLACK_US) {
                /*
                 * Frame playtime is already far behind the local DAC.
                 * Keep the PCM stream continuous and follow the current
                 * local timeline instead of dropping a burst of valid audio.
                 */
                drop = false;
            }

            if (atomic_load_explicit(
                    &flush_requested,
                    memory_order_acquire)) {
                continue;
            }

            if (drop) {
                atomic_store_explicit(
                    &audio_buf.read_idx,
                    ring_next(candidate_idx),
                    memory_order_release);

                atomic_fetch_add_explicit(
                    &out_stats.drops,
                    1,
                    memory_order_relaxed);

            } else {
                memcpy(
                    s_output_copy,
                    frame->data,
                    out_len);

                atomic_store_explicit(
                    &audio_buf.read_idx,
                    ring_next(candidate_idx),
                    memory_order_release);

                have_frame = true;
            }

            if (candidate_generation !=
                atomic_load_explicit(
                    &flush_generation,
                    memory_order_acquire)) {
                have_frame = false;
                continue;
            }
        }

        /* -------------------------------------------------------------
         * No frame.
         * ------------------------------------------------------------- */

        if (!have_frame) {
            vTaskDelay(1);
            continue;
        }

        /* -------------------------------------------------------------
         * DAC underrun accounting.
         * ------------------------------------------------------------- */

        int64_t current_us =
            esp_timer_get_time();

        if (dac_end_us != 0 &&
            dac_end_us < current_us - 2000 &&
            current_us >= underrun_grace_until_us) {
            atomic_fetch_add_explicit(
                &out_stats.underruns,
                1,
                memory_order_relaxed);
        }

        if (dac_end_us > current_us)
            start_us = dac_end_us;
        else
            start_us = current_us;

        /* -------------------------------------------------------------
         * Optional alignment silence.
         * ------------------------------------------------------------- */

        if (pad) {
            atomic_fetch_add_explicit(
                &out_stats.pads,
                1,
                memory_order_relaxed);

            write_silence_us(pad_us);
            start_us += pad_us;

        } else {
            int adj = 0;

#if DRIFT_COMPENSATION_ENABLED
            if (dac_end_us != 0) {
                int64_t level_us =
                    dac_end_us - current_us;

                adj =
                    drift_step(
                        level_us,
                        (uint32_t)(current_us / 1000));
            }
#endif

            if (adj > 0 &&
                out_len + SAMPLE_BYTES <= MAX_FRAME_SIZE) {

                memcpy(
                    s_output_copy + out_len,
                    s_output_copy + out_len - SAMPLE_BYTES,
                    SAMPLE_BYTES);

                out_len += SAMPLE_BYTES;

            } else if (
                adj < 0 &&
                out_len > SAMPLE_BYTES) {

                out_len -= SAMPLE_BYTES;
            }
        }

        /* -------------------------------------------------------------
         * Last flush check before I2S.
         * ------------------------------------------------------------- */

        if (atomic_load_explicit(
                &flush_requested,
                memory_order_acquire)) {
            continue;
        }

        if (audio_buf.output_cb) {
            audio_buf.output_cb(
                s_output_copy,
                out_len,
                audio_buf.user_ctx);
        }

        dac_end_us =
            start_us + frame_us(out_len);

        /* -------------------------------------------------------------
         * Statistics.
         * ------------------------------------------------------------- */

        uint32_t now_ms = gettime_ms();

        if (now_ms - out_stats.last_log_ms >= 5000) {

            uint32_t underruns =
                atomic_load_explicit(
                    &out_stats.underruns,
                    memory_order_relaxed);

            uint32_t pads =
                atomic_load_explicit(
                    &out_stats.pads,
                    memory_order_relaxed);

            uint32_t drops =
                atomic_load_explicit(
                    &out_stats.drops,
                    memory_order_relaxed);

            uint32_t overruns =
                atomic_load_explicit(
                    &out_stats.overruns,
                    memory_order_relaxed);

            uint32_t tap_calls =
                atomic_load_explicit(
                    &out_stats.tap_calls,
                    memory_order_relaxed);

            uint32_t dsp_total_us =
                atomic_load_explicit(
                    &out_stats.dsp_total_us,
                    memory_order_relaxed);

            uint32_t dsp_max_us =
                atomic_load_explicit(
                    &out_stats.dsp_max_us,
                    memory_order_relaxed);

            uint32_t dsp_avg_us =
                tap_calls ? (dsp_total_us / tap_calls) : 0;

            uint32_t dsp_total_samples =
                atomic_load_explicit(
                    &out_stats.dsp_total_samples,
                    memory_order_relaxed);

            uint32_t dsp_avg_samples =
                tap_calls ? (dsp_total_samples / tap_calls) : 0;

            uint32_t dsp_budget_us =
                (uint32_t)(((uint64_t)dsp_avg_samples * 1000000ULL) /
                           SAMPLE_RATE_HZ);

            uint32_t dsp_waits =
                atomic_load_explicit(
                    &out_stats.dsp_waits,
                    memory_order_relaxed);

            uint32_t dsp_drops =
                atomic_load_explicit(
                    &out_stats.dsp_drops,
                    memory_order_relaxed);

            int dma_lead =
                (int)(
                    (dac_end_us -
                     (int64_t)esp_timer_get_time()) /
                    1000);

            ESP_LOGI(
                TAG,
                "output 5s: "
                "underruns=%u "
                "pads=%u "
                "drops=%u "
                "overruns=%u "
                "tap=%u "
                "dsp_avg=%u us "
                "dsp_max=%u us "
                "frame=%u samples "
                "budget=%u us "
                "dsp_wait=%u "
                "dsp_drop=%u "
                "dma_lead=%d ms "
                "drift+=%u "
                "drift-=%u "
                "ema=%d ms",
                underruns,
                pads,
                drops,
                overruns,
                tap_calls,
                dsp_avg_us,
                dsp_max_us,
                dsp_avg_samples,
                dsp_budget_us,
                dsp_waits,
                dsp_drops,
                dma_lead,
                drift_inserts,
                drift_drops,
                (int)(drift_ema_us / 1000));

            stats_reset();
            drift_inserts = 0;
            drift_drops = 0;
        }
    }

    ESP_LOGD(
        TAG,
        "Audio output task exiting");

    if (audio_buf.task_done)
        xSemaphoreGive(audio_buf.task_done);

    vTaskDelete(NULL);
}

/* ========================================================================
 * Init
 * ======================================================================== */

void audio_buffer_init(
    audio_output_callback_t output_cb,
    void *user_ctx,
    audio_pcm_tap_cb_t tap_cb)
{
    if (audio_buf.frames) {
        audio_buf.output_cb = output_cb;
        audio_buf.user_ctx = user_ctx;
        audio_buf.tap_cb = tap_cb;
        audio_buf.tap_user_ctx = user_ctx;
        return;
    }

    memset(&audio_buf, 0, sizeof(audio_buf));

    audio_buf.output_cb = output_cb;
    audio_buf.user_ctx = user_ctx;
    audio_buf.tap_cb = tap_cb;
    audio_buf.tap_user_ctx = user_ctx;

    audio_buf.frames = heap_caps_malloc(
        BUFFER_FRAMES * sizeof(audio_frame_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!audio_buf.frames) {
        ESP_LOGE(TAG, "Failed to allocate raw ring in PSRAM");
        return;
    }

    memset(
        audio_buf.frames,
        0,
        BUFFER_FRAMES * sizeof(audio_frame_t));

    audio_buf.task_done =
        xSemaphoreCreateBinary();

    if (!audio_buf.task_done) {
        ESP_LOGE(TAG, "Failed to create task_done semaphore");
        free(audio_buf.frames);
        audio_buf.frames = NULL;
        return;
    }

    audio_buf.dsp_done =
        xSemaphoreCreateBinary();

    if (!audio_buf.dsp_done) {
        ESP_LOGE(TAG, "Failed to create dsp_done semaphore");
        vSemaphoreDelete(audio_buf.task_done);
        audio_buf.task_done = NULL;
        free(audio_buf.frames);
        audio_buf.frames = NULL;
        return;
    }

    dsp_frames = heap_caps_malloc(
        DSP_PIPE_FRAMES * sizeof(dsp_frame_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!dsp_frames) {
        ESP_LOGE(TAG, "Failed to allocate DSP pipeline in PSRAM");
        vSemaphoreDelete(audio_buf.dsp_done);
        audio_buf.dsp_done = NULL;
        vSemaphoreDelete(audio_buf.task_done);
        audio_buf.task_done = NULL;
        free(audio_buf.frames);
        audio_buf.frames = NULL;
        return;
    }

    memset(
        dsp_frames,
        0,
        DSP_PIPE_FRAMES * sizeof(dsp_frame_t));

    atomic_store_explicit(&audio_buf.read_idx, 0, memory_order_relaxed);
    atomic_store_explicit(&audio_buf.write_idx, 0, memory_order_relaxed);
    atomic_store_explicit(&dsp_read_idx, 0, memory_order_relaxed);
    atomic_store_explicit(&dsp_write_idx, 0, memory_order_relaxed);
    atomic_store_explicit(&flush_requested, false, memory_order_relaxed);
    atomic_store_explicit(&flush_generation, 0, memory_order_relaxed);
    atomic_store_explicit(&timing_correction.skip_frames, 0, memory_order_relaxed);
    atomic_store_explicit(&timing_correction.pause_frames, 0, memory_order_relaxed);

    stats_reset();

    ESP_LOGI(
        TAG,
        "Audio buffer initialized "
        "(SPSC raw=%u frames, ~%u KB PSRAM, "
        "SPSC DSP=%u frames, ~%u KB)",
        BUFFER_FRAMES,
        (unsigned)(BUFFER_FRAMES * sizeof(audio_frame_t) / 1024),
        DSP_PIPE_FRAMES,
        (unsigned)(DSP_PIPE_FRAMES * sizeof(dsp_frame_t) / 1024));
}

/* ========================================================================
 * Start
 * ======================================================================== */

void audio_buffer_start(void)
{
    if (!audio_buf.frames || !audio_buf.task_done) {
        ESP_LOGE(TAG, "Cannot start: buffer not initialized");
        return;
    }

    if (audio_buf.running) {
        ESP_LOGW(TAG, "Audio buffer already running");
        return;
    }

    atomic_store_explicit(&audio_buf.read_idx, 0, memory_order_relaxed);
    atomic_store_explicit(&audio_buf.write_idx, 0, memory_order_relaxed);
    atomic_store_explicit(&dsp_read_idx, 0, memory_order_relaxed);
    atomic_store_explicit(&dsp_write_idx, 0, memory_order_relaxed);

    atomic_store_explicit(&flush_requested, false, memory_order_relaxed);
    atomic_fetch_add_explicit(
        &flush_generation,
        1U,
        memory_order_acq_rel);

    dac_end_us = 0;
    underrun_grace_until_us = 0;
    stats_reset();

    drift_ema_us = 0;
    drift_ema_init = false;
    drift_last_adjust_ms = 0;
    drift_inserts = 0;
    drift_drops = 0;

    atomic_store_explicit(
        &timing_correction.skip_frames,
        0,
        memory_order_relaxed);

    atomic_store_explicit(
        &timing_correction.pause_frames,
        0,
        memory_order_relaxed);

    audio_buf.start_time = 0;
    audio_buf.running = true;

    while (xSemaphoreTake(audio_buf.task_done, 0) == pdTRUE) {
    }

    if (audio_buf.dsp_done) {
        while (xSemaphoreTake(audio_buf.dsp_done, 0) == pdTRUE) {
        }
    }

    if (audio_buf.tap_cb) {
        BaseType_t rc =
            xTaskCreatePinnedToCore(
                audio_dsp_task,
                "audio_dsp",
                DSP_TASK_STACK_SIZE,
                NULL,
                DSP_TASK_PRIORITY,
                &audio_buf.dsp_task,
                DSP_TASK_CORE);

        if (rc != pdPASS) {
            ESP_LOGE(TAG, "Failed to create DSP task");
            audio_buf.dsp_task = NULL;
            audio_buf.running = false;
            return;
        }
    }

    BaseType_t rc =
        xTaskCreatePinnedToCore(
            audio_output_task,
            "audio_output",
            AUDIO_TASK_STACK_SIZE,
            NULL,
            AUDIO_TASK_PRIORITY,
            &audio_buf.task,
            AUDIO_TASK_CORE);

    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio output task");
        audio_buf.running = false;

        if (audio_buf.dsp_task) {
            if (audio_buf.dsp_done) {
                xSemaphoreTake(
                    audio_buf.dsp_done,
                    pdMS_TO_TICKS(AUDIO_STOP_WAIT_MS));
            }
            audio_buf.dsp_task = NULL;
        }

        return;
    }

    ESP_LOGI(
        TAG,
        "Audio playback started (DSP=%s, lock-free SPSC, output core=%d priority=%d)",
        audio_buf.tap_cb ? "ON" : "OFF",
        AUDIO_TASK_CORE,
        AUDIO_TASK_PRIORITY);
}

/* ========================================================================
 * Start time
 * ======================================================================== */

void audio_buffer_set_start_time(uint32_t playtime)
{
    if (!audio_buf.running)
        return;

    audio_buf.start_time = playtime;
    underrun_grace_until_us =
        esp_timer_get_time() +
        (UNDERRUN_STARTUP_GRACE_MS * 1000LL);

    if (audio_buf.task)
        xTaskNotifyGive(audio_buf.task);
}

/* ========================================================================
 * Stop
 * ======================================================================== */

void audio_buffer_stop(void)
{
    if (!audio_buf.running)
        return;

    audio_buf.running = false;

    if (audio_buf.task)
        xTaskNotifyGive(audio_buf.task);

    if (audio_buf.task_done) {
        if (xSemaphoreTake(
                audio_buf.task_done,
                pdMS_TO_TICKS(AUDIO_STOP_WAIT_MS)) != pdTRUE) {

            ESP_LOGW(TAG, "Output task stop timeout; deleting task");

            if (audio_buf.task)
                vTaskDelete(audio_buf.task);
        }
    }

    audio_buf.task = NULL;

    if (audio_buf.dsp_task) {

        if (audio_buf.dsp_done) {
            if (xSemaphoreTake(
                    audio_buf.dsp_done,
                    pdMS_TO_TICKS(AUDIO_STOP_WAIT_MS)) != pdTRUE) {

                ESP_LOGW(TAG, "DSP task stop timeout; deleting task");

                if (audio_buf.dsp_task)
                    vTaskDelete(audio_buf.dsp_task);
            }
        }

        audio_buf.dsp_task = NULL;
    }

    ESP_LOGI(TAG, "Audio playback stopped");
}

/* ========================================================================
 * RTP producer: raw SPSC writer
 * ======================================================================== */

bool audio_buffer_write(
    const uint8_t *data,
    size_t len,
    uint32_t playtime)
{
    if (!audio_buf.frames || !data || !audio_buf.running)
        return false;

    if (len == 0 || len > MAX_FRAME_SIZE)
        return false;

    /* -------------------------------------------------------------
     * Producer owns write_idx.
     * Consumer owns read_idx.
     * No mutex, no retry loop, no scheduler blocking.
     * ------------------------------------------------------------- */

    uint32_t write_idx =
        atomic_load_explicit(
            &audio_buf.write_idx,
            memory_order_relaxed);

    uint32_t read_idx =
        atomic_load_explicit(
            &audio_buf.read_idx,
            memory_order_acquire);

    uint32_t next_write =
        ring_next(write_idx);

    if (next_write == read_idx) {
        atomic_fetch_add_explicit(
            &out_stats.overruns,
            1,
            memory_order_relaxed);

        return false;
    }

    audio_frame_t *frame =
        &audio_buf.frames[write_idx];

    memcpy(frame->data, data, len);

    frame->len = len;
    frame->playtime = playtime;

    /* Publish PCM only after all frame fields are written. */
    atomic_store_explicit(
        &audio_buf.write_idx,
        next_write,
        memory_order_release);

    return true;
}

/* ========================================================================
 * Flush request
 * ======================================================================== */

void audio_buffer_flush(void)
{
    if (!audio_buf.frames)
        return;

    /*
     * Do not touch either ring index here.  The task owning the consumer
     * side performs the reset, preserving true SPSC ownership.
     */
    atomic_store_explicit(
        &flush_requested,
        true,
        memory_order_release);

    dac_end_us = 0;
    audio_buf.start_time = 0;

    atomic_store_explicit(
        &timing_correction.skip_frames,
        0,
        memory_order_relaxed);

    atomic_store_explicit(
        &timing_correction.pause_frames,
        0,
        memory_order_relaxed);

    if (audio_buf.task)
        xTaskNotifyGive(audio_buf.task);

    ESP_LOGD(TAG, "Buffer flush requested");
}

/* ========================================================================
 * Deinit
 * ======================================================================== */

void audio_buffer_deinit(void)
{
    audio_buffer_stop();

    if (dsp_frames) {
        free(dsp_frames);
        dsp_frames = NULL;
    }

    if (audio_buf.dsp_done) {
        vSemaphoreDelete(audio_buf.dsp_done);
        audio_buf.dsp_done = NULL;
    }

    if (audio_buf.task_done) {
        vSemaphoreDelete(audio_buf.task_done);
        audio_buf.task_done = NULL;
    }

    if (audio_buf.frames) {
        free(audio_buf.frames);
        audio_buf.frames = NULL;
    }

    audio_buf.output_cb = NULL;
    audio_buf.user_ctx = NULL;
    audio_buf.tap_cb = NULL;
    audio_buf.tap_user_ctx = NULL;
    audio_buf.task = NULL;
    audio_buf.dsp_task = NULL;

    atomic_store_explicit(&flush_requested, false, memory_order_relaxed);

    ESP_LOGI(TAG, "Audio buffer deinitialized");
}

/* ========================================================================
 * Timing snapshot
 * ======================================================================== */

void audio_buffer_get_timing(
    uint32_t *frames_buffered,
    uint32_t *head_playtime)
{
    if (!frames_buffered || !head_playtime)
        return;

    *frames_buffered = 0;
    *head_playtime = 0;

    if (!audio_buf.frames)
        return;

    if (atomic_load_explicit(
            &flush_requested,
            memory_order_acquire)) {
        return;
    }

    /* -------------------------------------------------------------
     * Raw SPSC snapshot.
     * ------------------------------------------------------------- */

    uint32_t raw_r =
        atomic_load_explicit(
            &audio_buf.read_idx,
            memory_order_relaxed);

    uint32_t raw_w =
        atomic_load_explicit(
            &audio_buf.write_idx,
            memory_order_acquire);

    uint32_t raw_count =
        raw_ring_count(raw_r, raw_w);

    uint32_t raw_head_playtime = 0;

    if (raw_count > 0) {
        uint32_t idx =
            (raw_w == 0)
                ? BUFFER_FRAMES - 1
                : raw_w - 1;

        raw_head_playtime =
            audio_buf.frames[idx].playtime;
    }

    /* -------------------------------------------------------------
     * DSP SPSC snapshot.
     * ------------------------------------------------------------- */

    uint32_t dsp_count = 0;
    uint32_t dsp_head_playtime = 0;

    if (audio_buf.tap_cb && dsp_frames) {

        uint32_t dsp_r =
            atomic_load_explicit(
                &dsp_read_idx,
                memory_order_relaxed);

        uint32_t dsp_w =
            atomic_load_explicit(
                &dsp_write_idx,
                memory_order_acquire);

        dsp_count =
            dsp_ring_count(dsp_r, dsp_w);

        if (dsp_count > 0) {
            uint32_t idx =
                (dsp_w == 0)
                    ? DSP_PIPE_FRAMES - 1
                    : dsp_w - 1;

            dsp_head_playtime =
                dsp_frames[idx].playtime;
        }
    }

    *frames_buffered =
        raw_count + dsp_count;

    /* Processed queue is further downstream, so prefer its newest head. */
    if (dsp_count > 0) {
        *head_playtime = dsp_head_playtime;
    } else if (raw_count > 0) {
        *head_playtime = raw_head_playtime;
    }
}

/* ========================================================================
 * External timing correction
 * ======================================================================== */

void audio_buffer_skip_frames(uint32_t count)
{
    /*
     * Keep the PCM path continuous. The legacy external frame-skip
     * correction is intentionally disabled; RTP playtime + local DAC
     * drift compensation handle timing without deleting PCM frames.
     */
    (void)count;

    atomic_store_explicit(
        &timing_correction.skip_frames,
        0,
        memory_order_release);
}

void audio_buffer_pause_frames(uint32_t count)
{
    atomic_store_explicit(
        &timing_correction.pause_frames,
        count,
        memory_order_release);
}

/* ========================================================================
 * Ready
 * ======================================================================== */

bool audio_buffer_is_ready(void)
{
    return
        audio_buf.frames != NULL &&
        audio_buf.running;
}
