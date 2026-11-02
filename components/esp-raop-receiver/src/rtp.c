
/*
 * HairTunes - RAOP packet handler and slave-clocked replay engine
 * Copyright (c) James Laird 2011
 * All rights reserved.
 *
 * Modularisation: philippe_44@outlook.com, 2019
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/types.h>
#include <pthread.h>
#include <math.h>
#include <errno.h>
#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>
#include <assert.h>
#include "platform.h"
#include "rtp.h"
#include "log_util.h"
#include "util.h"

#ifdef WIN32
#include <openssl/aes.h>
#include "alac_wrapper.h"
#define MSG_DONTWAIT 0
#else
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <mbedtls/version.h>
#include <mbedtls/aes.h>
#include "alac_wrapper.h"
#endif

#define NTP2MS(ntp) ((((ntp) >> 10) * 1000L) >> 22)
#define MS2NTP(ms) (((((u64_t) (ms)) << 22) / 1000) << 10)
#define NTP2TS(ntp, rate) ((((ntp) >> 16) * (rate)) >> 16)
#define TS2NTP(ts, rate)  (((((u64_t) (ts)) << 16) / (rate)) << 16)
#define MS2TS(ms, rate) ((((u64_t) (ms)) * (rate)) / 1000)
#define TS2MS(ts, rate) NTP2MS(TS2NTP(ts,rate))

extern log_level 	raop_loglevel;

//#define __RTP_STORE

// default buffer size
#define BUFFER_FRAMES_MAX 	((RAOP_SAMPLE_RATE * 10) / 352 )
#define BUFFER_FRAMES_MIN 	( (150 * RAOP_SAMPLE_RATE * 2) / (352 * 100) )
#define MAX_PACKET       1408
#define MIN_LATENCY         11025
#define MAX_LATENCY         ((120 * RAOP_SAMPLE_RATE * 2) / 100)

#define RTP_STACK_SIZE	(6*1024)

#define RTP_SYNC	(0x01)
#define NTP_SYNC	(0x02)

#define RESEND_TO           250
#define MISSING_GRACE       100
#define RTP_PREBUFFER_MS    1500
#define RTP_RX_DRAIN_MAX    16
#define RTP_PUSH_MAX_FRAMES 8
#define RTP_HOLE_SCAN_MAX   96
#define RTP_RESEND_MAX_RANGE 8
#define RTP_LATE_TOLERANCE_MS  20

#define RTP_PREBUFFER_FRAMES   ((RTP_PREBUFFER_MS * RAOP_SAMPLE_RATE) / 1000)

enum { DATA = 0, CONTROL, TIMING };

uint32_t buffer_frames = ((150 * RAOP_SAMPLE_RATE * 2) / (352 * 100));

typedef u16_t seq_t;
typedef struct __attribute__((__packed__)) audio_buffer_entry {
    u32_t rtptime;
    u32_t last_resend;
    u32_t generation;
    u16_t seqno;
    s16_t *data;
    u16_t len;
    u8_t ready;
    u8_t allocated;
    u8_t missed;
    u8_t decoding;
} abuf_t;

typedef struct rtp_s {

#ifdef __RTP_STORE
	FILE *rtpIN, *rtpOUT;
#endif
	bool running;
	unsigned char aesiv[16];
#ifdef WIN32
	AES_KEY aes;
#else
	mbedtls_aes_context aes;
#endif
	
	bool decrypt;
	u8_t *decrypt_buf;
	u32_t last_resend_request;
	u32_t frame_size, frame_duration;
	u32_t in_frames, out_frames;
	struct in_addr host;
	struct sockaddr_in rtp_host;
	struct {
		unsigned short rport, lport;
		int sock;
	} rtp_sockets[3]; 					 // data, control, timing
	struct timing_s {
		u64_t local, remote;
	} timing;
	struct {
		u32_t 	rtp, time;
		u8_t  	status;
	} synchro;
	int latency;			// rtp hold depth in samples
	u32_t resent_req, resent_rec;	// total resent + recovered frames
	u32_t silent_frames;	// total silence frames
	u32_t discarded;
	abuf_t audio_buffer[BUFFER_FRAMES_MAX];
	seq_t ab_read, ab_write;
	pthread_mutex_t ab_mutex;
#ifdef WIN32
	pthread_t thread;
#else
	TaskHandle_t thread, joiner;
	StaticTask_t *xTaskBuffer;
    StackType_t xStack[RTP_STACK_SIZE] __attribute__ ((aligned (4)));
#endif

	struct alac_codec_s *alac_codec;
	int first_seqno;
	enum { RTP_WAIT, RTP_STREAM, RTP_PREBUFFER, RTP_PLAY } state;
    int stalled;
	raop_data_cb_t data_cb;
	raop_cmd_cb_t cmd_cb;
    uint8_t *user_buffer;
    uint8_t *decode_pcm;
    uint8_t *playback_pcm;
    u32_t generation;

    /* PREBUFFER stall tracking (see buffer_push_packet) */
    bool  prebuf_waiting;
    u32_t prebuf_start_ms;
} rtp_t;


#define BUFIDX(seqno) ((seq_t)(seqno) % buffer_frames)
static void 	buffer_alloc(abuf_t *audio_buffer, int size, uint8_t *buf, size_t buf_size);
static void 	buffer_release(abuf_t *audio_buffer);
static void 	buffer_reset(abuf_t *audio_buffer);
static void 	buffer_push_packet(rtp_t *ctx);
static void     request_missing(rtp_t *ctx);
static bool 	rtp_request_resend(rtp_t *ctx, seq_t first, seq_t last);
static bool 	rtp_request_timing(rtp_t *ctx);
static int	  	seq_order(seq_t a, seq_t b);
#ifdef WIN32
static void 	*rtp_thread_func(void *arg);
#else
static void 	rtp_thread_func(void *arg);
#endif

/*---------------------------------------------------------------------------*/
static struct alac_codec_s* alac_init(int fmtp[12]) {
	struct alac_codec_s *alac;
	unsigned sample_rate, block_size;
	unsigned char sample_size, channels;
	struct {
		uint32_t	frameLength;
		uint8_t		compatibleVersion;
		uint8_t		bitDepth;
		uint8_t		pb;
		uint8_t		mb;
		uint8_t		kb;
		uint8_t		numChannels;
		uint16_t	maxRun;
		uint32_t	maxFrameBytes;
		uint32_t	avgBitRate;
		uint32_t	sampleRate;
	} config;

	config.frameLength = htonl(fmtp[1]);
	config.compatibleVersion = fmtp[2];
	config.bitDepth = fmtp[3];
	config.pb = fmtp[4];
	config.mb = fmtp[5];
	config.kb = fmtp[6];
	config.numChannels = fmtp[7];
	config.maxRun = htons(fmtp[8]);
	config.maxFrameBytes = htonl(fmtp[9]);
	config.avgBitRate = htonl(fmtp[10]);
	config.sampleRate = htonl(fmtp[11]);

	alac = alac_create_decoder(sizeof(config), (unsigned char*) &config, &sample_size, &sample_rate, &channels, &block_size);
	if (!alac) {
		LOG_ERROR("cannot create alac codec", NULL);
		return NULL;
	}

	return alac;
}

/*---------------------------------------------------------------------------*/
rtp_resp_t rtp_init(struct in_addr host, int latency, char *aeskey, char *aesiv, char *fmtpstr,
								short unsigned pCtrlPort, short unsigned pTimingPort,
								uint8_t *buffer, size_t size,
								raop_cmd_cb_t cmd_cb, raop_data_cb_t data_cb)
{
	int i = 0;
	char *arg;
	int fmtp[12];
	bool rc = true;
	rtp_t *ctx = heap_caps_calloc(1, sizeof(rtp_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	rtp_resp_t resp = { 0, 0, 0, NULL };

	if (!ctx) return resp;

	ctx->host = host;
	ctx->decrypt = false;
	ctx->cmd_cb = cmd_cb;
	ctx->data_cb = data_cb;
	ctx->rtp_host.sin_family = AF_INET;
	ctx->rtp_host.sin_addr.s_addr = INADDR_ANY;
	pthread_mutex_init(&ctx->ab_mutex, 0);
	ctx->first_seqno = -1;
	ctx->latency = latency;
	ctx->ab_read = ctx->ab_write;
    ctx->generation = 1;

#ifdef __RTP_STORE
	ctx->rtpIN = fopen("airplay.rtpin", "wb");
	ctx->rtpOUT = fopen("airplay.rtpout", "wb");
#endif

	ctx->rtp_sockets[CONTROL].rport = pCtrlPort;
	ctx->rtp_sockets[TIMING].rport = pTimingPort;

	if (aesiv && aeskey) {
		memcpy(ctx->aesiv, aesiv, 16);
#ifdef WIN32
		AES_set_decrypt_key((unsigned char*) aeskey, 128, &ctx->aes);
#else
		memset(&ctx->aes, 0, sizeof(mbedtls_aes_context));
		mbedtls_aes_setkey_dec(&ctx->aes, (unsigned char*) aeskey, 128);
#endif
		ctx->decrypt = true;
		ctx->decrypt_buf = malloc(MAX_PACKET);
	}

	memset(fmtp, 0, sizeof(fmtp));
	while (i < (int)(sizeof(fmtp) / sizeof(fmtp[0])) &&
           (arg = strsep(&fmtpstr, " \t")) != NULL)
        fmtp[i++] = atoi(arg);

	ctx->frame_size = fmtp[1];
	ctx->frame_duration = (ctx->frame_size * 1000) / RAOP_SAMPLE_RATE;

	// alac decoder
    ctx->alac_codec = alac_init(fmtp);
    rc &= ctx->alac_codec != NULL;

    if (ctx->frame_size == 0 || ctx->frame_size * 4 > MAX_PACKET)
        rc = false;

    if (rc) {
        ctx->decode_pcm = heap_caps_malloc(ctx->frame_size * 4,
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ctx->playback_pcm = heap_caps_malloc(ctx->frame_size * 4,
                                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        rc &= (ctx->decode_pcm != NULL && ctx->playback_pcm != NULL);
    }

	buffer_alloc(ctx->audio_buffer, ctx->frame_size*4, buffer, size);
	ctx->user_buffer = buffer;

	// create rtp ports
	for (i = 0; i < 3; i++) {
		ctx->rtp_sockets[i].sock = bind_socket(&ctx->rtp_sockets[i].lport, SOCK_DGRAM);
		rc &= ctx->rtp_sockets[i].sock > 0;
	}

	// create http port and start listening
	resp.cport = ctx->rtp_sockets[CONTROL].lport;
	resp.tport = ctx->rtp_sockets[TIMING].lport;
	resp.aport = ctx->rtp_sockets[DATA].lport;

#ifdef WIN32
    ctx->running = true;
    if (pthread_create(&ctx->thread, NULL, rtp_thread_func, (void *)ctx) != 0) {
        ctx->running = false;
        rc = false;
    }
#else
    ctx->xTaskBuffer = (StaticTask_t *)heap_caps_malloc(
        sizeof(StaticTask_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!ctx->xTaskBuffer) {
        rc = false;
    } else {
        BaseType_t core_id =
            (CONFIG_PTHREAD_TASK_CORE_DEFAULT == -1) ?
            tskNO_AFFINITY : CONFIG_PTHREAD_TASK_CORE_DEFAULT;

        ctx->running = true;
        ctx->thread = xTaskCreateStaticPinnedToCore(
            (TaskFunction_t)rtp_thread_func,
            "RTP_thread",
            RTP_STACK_SIZE,
            ctx,
            CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT + 1,
            ctx->xStack,
            ctx->xTaskBuffer,
            core_id);

        if (!ctx->thread) {
            ctx->running = false;
            rc = false;
        }
    }
#endif


	// cleanup everything if we failed
	if (!rc) {
		LOG_ERROR("[%p]: cannot start RTP", ctx);
		rtp_end(ctx);
		ctx = NULL;
	}

	resp.ctx = ctx;
	return resp;
}

/*---------------------------------------------------------------------------*/
void rtp_end(rtp_t *ctx)
{
    int i;
    if (!ctx) return;

    if (ctx->running) {
#ifndef WIN32
        ctx->joiner = xTaskGetCurrentTaskHandle();
#endif
        ctx->running = false;
#ifdef WIN32
        pthread_join(ctx->thread, NULL);
#else
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        if (ctx->thread) vTaskDelete(ctx->thread);
        SAFE_PTR_FREE(ctx->xTaskBuffer);
#endif
    }

    for (i = 0; i < 3; i++) {
        if (ctx->rtp_sockets[i].sock > 0)
            closesocket(ctx->rtp_sockets[i].sock);
    }

    if (ctx->alac_codec) alac_delete_decoder(ctx->alac_codec);
    if (ctx->decrypt_buf) free(ctx->decrypt_buf);
    if (ctx->decode_pcm) free(ctx->decode_pcm);
    if (ctx->playback_pcm) free(ctx->playback_pcm);

    pthread_mutex_destroy(&ctx->ab_mutex);
    buffer_release(ctx->audio_buffer);
    if (ctx->user_buffer) free(ctx->user_buffer);

#ifdef __RTP_STORE
    if (ctx->rtpIN) fclose(ctx->rtpIN);
    if (ctx->rtpOUT) fclose(ctx->rtpOUT);
#endif

    free(ctx);
}

/*---------------------------------------------------------------------------*/
bool rtp_flush(rtp_t *ctx, unsigned short seqno, unsigned int rtptime, bool exit_locked)
{
    pthread_mutex_lock(&ctx->ab_mutex);
    bool flushed = (ctx->state == RTP_PLAY || ctx->state == RTP_PREBUFFER);

    ctx->generation++;
    ctx->first_seqno = seqno;
    buffer_reset(ctx->audio_buffer);
    ctx->state = RTP_WAIT;

    LOG_INFO("[%p]: FLUSH packets below %hu - %u", ctx, seqno, rtptime);

    if (!exit_locked || !flushed)
        pthread_mutex_unlock(&ctx->ab_mutex);

    return flushed;
}

/*---------------------------------------------------------------------------*/
void rtp_flush_release(rtp_t *ctx)
{
    pthread_mutex_unlock(&ctx->ab_mutex);
}

/*---------------------------------------------------------------------------*/
void rtp_record(rtp_t *ctx, unsigned short seqno, unsigned rtptime)
{
    pthread_mutex_lock(&ctx->ab_mutex);
    ctx->generation++;
    ctx->first_seqno = (seqno || rtptime) ? seqno : -1;
    ctx->state = RTP_WAIT;
    ctx->stalled = 0;
    pthread_mutex_unlock(&ctx->ab_mutex);
    LOG_DEBUG("[%p]: record %hu - %u", ctx, seqno, rtptime);
}

/*---------------------------------------------------------------------------*/
static void buffer_alloc(abuf_t *audio_buffer, int size, uint8_t *buf, size_t buf_size)
{
    for (buffer_frames = 0;
         buf && buf_size >= (size_t)size && buffer_frames < BUFFER_FRAMES_MAX;
         buffer_frames++) {
        memset(&audio_buffer[buffer_frames], 0, sizeof(abuf_t));
        audio_buffer[buffer_frames].data = (s16_t *)buf;
        buf += size;
        buf_size -= size;
    }
    LOG_INFO("allocated %d buffers (min=%d) from buffer of %zu bytes",
             buffer_frames, BUFFER_FRAMES_MIN,
             buf_size + (size_t)buffer_frames * size);
    for (; buffer_frames < BUFFER_FRAMES_MIN; buffer_frames++) {
        memset(&audio_buffer[buffer_frames], 0, sizeof(abuf_t));
        audio_buffer[buffer_frames].data = malloc(size);
        audio_buffer[buffer_frames].allocated = 1;
    }
}

/*---------------------------------------------------------------------------*/
static void buffer_release(abuf_t *audio_buffer) {
	int i;
	for (i = 0; i < buffer_frames; i++) {
		if (audio_buffer[i].allocated) free(audio_buffer[i].data);
	}
}

/*---------------------------------------------------------------------------*/
static void buffer_reset(abuf_t *audio_buffer)
{
    int i;
    for (i = 0; i < buffer_frames; i++) {
        audio_buffer[i].ready = 0;
        audio_buffer[i].missed = 0;
        audio_buffer[i].decoding = 0;
        audio_buffer[i].last_resend = 0;
        audio_buffer[i].generation = 0;
        audio_buffer[i].seqno = 0;
    }
}

/*---------------------------------------------------------------------------*/
// the sequence numbers will wrap pretty often.
// this returns true if the second arg is after the first
static int seq_order(seq_t a, seq_t b) {
	s16_t d = b - a;
	return d > 0;
}

/*---------------------------------------------------------------------------*/
static void alac_decode(rtp_t *ctx, s16_t *dest, char *buf, int len, u16_t *outsize) {
	unsigned char iv[16];
	int aeslen;
	assert(len<=MAX_PACKET);

	if (ctx->decrypt) {
		aeslen = len & ~0xf;
		memcpy(iv, ctx->aesiv, sizeof(iv));
#ifdef WIN32
		AES_cbc_encrypt((unsigned char*)buf, ctx->decrypt_buf, aeslen, &ctx->aes, iv, AES_DECRYPT);
#else
		mbedtls_aes_crypt_cbc(&ctx->aes, MBEDTLS_AES_DECRYPT, aeslen, iv, (unsigned char*) buf, ctx->decrypt_buf);
#endif
		memcpy(ctx->decrypt_buf+aeslen, buf+aeslen, len-aeslen);
		alac_to_pcm(ctx->alac_codec, (unsigned char*) ctx->decrypt_buf, (unsigned char*) dest, 2, (unsigned int*) outsize);
	} else {
		alac_to_pcm(ctx->alac_codec, (unsigned char*) buf, (unsigned char*) dest, 2, (unsigned int*) outsize);
	}

	*outsize *= 4;
}


/*---------------------------------------------------------------------------*/
static void buffer_put_packet(rtp_t *ctx, seq_t seqno, unsigned rtptime,
                              bool first, char *data, int len, bool push)
{
    abuf_t *abuf = NULL;
    u32_t generation = 0;
    u16_t out_len = 0;
    bool decode_ok = false;

    pthread_mutex_lock(&ctx->ab_mutex);

    if (ctx->first_seqno != -1 && seq_order(seqno, ctx->first_seqno)) {
        pthread_mutex_unlock(&ctx->ab_mutex);
        return;
    }

    if (ctx->state == RTP_WAIT) {
        ctx->ab_write = seqno - 1;
        ctx->ab_read = ctx->ab_write + 1;
        ctx->resent_req = 0;
        ctx->resent_rec = 0;
        ctx->silent_frames = 0;
        ctx->discarded = 0;

        if (ctx->first_seqno != -1) {
            ctx->state = RTP_PREBUFFER;
            ctx->first_seqno = -1;
            ctx->prebuf_waiting = false;
            LOG_INFO("[%p]: 1st accepted packet:%hu, entering PREBUFFER", ctx, seqno);
        } else {
            ctx->state = RTP_STREAM;
            LOG_INFO("[%p]: 1st accepted packet:%hu, waiting for FLUSH", ctx, seqno);
        }
    } else if (ctx->state == RTP_STREAM && ctx->first_seqno != -1 &&
               seq_order(ctx->first_seqno, seqno + 1)) {
        while (seq_order(ctx->ab_read, ctx->first_seqno)) {
            abuf_t *old = ctx->audio_buffer + BUFIDX(ctx->ab_read);
            old->ready = 0;
            old->missed = 1;
            old->decoding = 0;
            ctx->ab_read++;
        }
        ctx->state = RTP_PREBUFFER;
        ctx->first_seqno = -1;
        ctx->prebuf_waiting = false;
        LOG_INFO("[%p]: done waiting for FLUSH with packet:%hu, entering PREBUFFER starting:%hu",
                 ctx, seqno, ctx->ab_read);
    }

    abuf = ctx->audio_buffer + BUFIDX(seqno);

    if (seqno == (u16_t)(ctx->ab_write + 1)) {
        ctx->ab_write = seqno;
    } else if (seq_order(ctx->ab_write, seqno)) {
        u16_t gap = (u16_t)(seqno - ctx->ab_write - 1);

        if (gap >= buffer_frames ||
            (ctx->latency > 0 && gap > (u16_t)(ctx->latency / ctx->frame_size))) {
            LOG_WARN("[%p]: large RTP gap=%hu at seq=%hu, resetting ring",
                     ctx, gap, seqno);
            buffer_reset(ctx->audio_buffer);
            ctx->ab_read = seqno;
        } else {
            for (seq_t i = ctx->ab_write + 1; seq_order(i, seqno); i++) {
                abuf_t *missing = ctx->audio_buffer + BUFIDX(i);
                missing->ready = 0;
                missing->missed = 0;
                missing->decoding = 0;
                missing->generation = ctx->generation;
                missing->seqno = i;
                missing->rtptime = rtptime - (seqno - i) * ctx->frame_size;
                missing->last_resend = 0;
            }
        }
        ctx->ab_write = seqno;
    } else if (seq_order(ctx->ab_read, seqno + 1)) {
        ctx->resent_rec++;
    } else {
        if (abuf->missed)
            LOG_DEBUG("[%p]: packet too late seqno:%hu rtptime:%u (W:%hu R:%hu)",
                      ctx, seqno, rtptime, ctx->ab_write, ctx->ab_read);
        abuf = NULL;
    }

    if (abuf) {
        generation = ctx->generation;
        abuf->seqno = seqno;
        abuf->generation = generation;
        abuf->decoding = 1;
        abuf->ready = 0;
        abuf->missed = 0;
        abuf->rtptime = rtptime;
    }

    pthread_mutex_unlock(&ctx->ab_mutex);

    if (!abuf || !ctx->decode_pcm)
        return;

    alac_decode(ctx, (s16_t *)ctx->decode_pcm, data, len, &out_len);
    decode_ok = (out_len == ctx->frame_size * 4);

    pthread_mutex_lock(&ctx->ab_mutex);
    if (abuf->generation == generation &&
        abuf->seqno == seqno &&
        abuf->decoding) {
        abuf->decoding = 0;
        if (decode_ok) {
            memcpy(abuf->data, ctx->decode_pcm, out_len);
            abuf->len = out_len;
            abuf->rtptime = rtptime;
            abuf->last_resend = 0;
            abuf->missed = 0;
            abuf->ready = 1;
        } else {
            abuf->ready = 0;
            abuf->missed = 1;
            LOG_WARN("[%p]: ALAC anomaly seq:%hu in:%d out:%hu expected:%u",
                     ctx, seqno, len, out_len, ctx->frame_size * 4);
        }
    }
    pthread_mutex_unlock(&ctx->ab_mutex);

#ifdef __RTP_STORE
    if (decode_ok) {
        fwrite(data, len, 1, ctx->rtpIN);
        fwrite(ctx->decode_pcm, out_len, 1, ctx->rtpOUT);
    }
#endif

    if (push)
        buffer_push_packet(ctx);
}

/*---------------------------------------------------------------------------*/
static void request_missing(rtp_t *ctx)
{
    seq_t first = 0, last = 0;
    bool found = false;
    u16_t span, scan;
    u32_t now = gettime_ms();

    pthread_mutex_lock(&ctx->ab_mutex);
    span = (u16_t)(ctx->ab_write - ctx->ab_read);
    if (span == 0) {
        pthread_mutex_unlock(&ctx->ab_mutex);
        return;
    }
    scan = span > RTP_HOLE_SCAN_MAX ? RTP_HOLE_SCAN_MAX : span;

    for (u16_t off = 0; off <= scan; off++) {
        seq_t seq = ctx->ab_read + off;
        abuf_t *f = ctx->audio_buffer + BUFIDX(seq);
        if (f->ready || f->decoding)
            continue;
        if (f->last_resend && now - f->last_resend < RESEND_TO)
            continue;
        if (!found) {
            first = last = seq;
            found = true;
        } else if ((u16_t)(seq - last) == 1 &&
                   (u16_t)(last - first + 1) < RTP_RESEND_MAX_RANGE) {
            last = seq;
        } else {
            break;
        }
    }

    u32_t generation = ctx->generation;
    pthread_mutex_unlock(&ctx->ab_mutex);

    if (found && rtp_request_resend(ctx, first, last)) {
        pthread_mutex_lock(&ctx->ab_mutex);
        if (ctx->generation == generation) {
            for (seq_t seq = first;; seq++) {
                ctx->audio_buffer[BUFIDX(seq)].last_resend = now;
                if (seq == last)
                    break;
            }
        }
        pthread_mutex_unlock(&ctx->ab_mutex);
    }
}

/*---------------------------------------------------------------------------*/
static void buffer_push_packet(rtp_t *ctx)
{
    u32_t hold;
    u32_t pushed = 0;

    if (!ctx || !ctx->data_cb)
        return;
    if (ctx->state != RTP_PLAY && ctx->state != RTP_PREBUFFER)
        return;
    if (ctx->synchro.status != (RTP_SYNC | NTP_SYNC))
        return;

    hold = max((ctx->latency * 1000) / (8 * RAOP_SAMPLE_RATE), 100);

    if (ctx->state == RTP_PREBUFFER) {
        u32_t playtime = 0;
        bool start = false;
        pthread_mutex_lock(&ctx->ab_mutex);
        seq_t read_seq = ctx->ab_read;
        u32_t frames = 0;
        seq_t s = read_seq;
        while (seq_order(s, ctx->ab_write) || s == ctx->ab_write) {
            abuf_t *f = ctx->audio_buffer + BUFIDX(s);
            if (!f->ready)
                break;
            frames += ctx->frame_size;
            if (frames >= RTP_PREBUFFER_FRAMES) {
                abuf_t *head = ctx->audio_buffer + BUFIDX(read_seq);
                playtime = ctx->synchro.time +
                    ((head->rtptime - ctx->synchro.rtp) * 10) /
                    (RAOP_SAMPLE_RATE / 100);
                ctx->state = RTP_PLAY;
                start = true;
                break;
            }
            s++;
        }

        /*
         * Fallback: a single unrecovered hole near ab_read used to block
         * PREBUFFER forever (request_missing() kept being called with no
         * timeout, while ab_write kept advancing until the u16 sequence
         * number lapped ab_read -- "PREBUFFER READY: 95428 ms" / W < R).
         *
         * Once the buffer already SPANS the prebuffer time and most of it
         * is present, start anyway: PLAY-side logic already knows how to
         * skip a remaining hole.
         */
        if (!start) {
            u16_t span = (u16_t)(ctx->ab_write - ctx->ab_read + 1);

            if ((u32_t)span * ctx->frame_size >= RTP_PREBUFFER_FRAMES &&
                span < buffer_frames) {

                u32_t ready = 0;

                for (u16_t i = 0; i < span; i++) {
                    if ((ctx->audio_buffer +
                         BUFIDX(ctx->ab_read + i))->ready)
                        ready++;
                }

                if (ready * 2 >= span) {
                    /* Drop any leading holes so playback starts on data. */
                    while (ctx->ab_read != ctx->ab_write &&
                           !(ctx->audio_buffer +
                             BUFIDX(ctx->ab_read))->ready) {
                        ctx->ab_read++;
                    }

                    abuf_t *head =
                        ctx->audio_buffer + BUFIDX(ctx->ab_read);

                    if (head->ready) {
                        playtime = ctx->synchro.time +
                            ((head->rtptime - ctx->synchro.rtp) * 10) /
                            (RAOP_SAMPLE_RATE / 100);
                        ctx->state = RTP_PLAY;
                        start = true;

                        LOG_WARN("[%p]: PREBUFFER stuck on a hole after "
                                 "%u ms, starting anyway (span:%hu ready:%u)",
                                 ctx,
                                 ctx->prebuf_waiting
                                     ? gettime_ms() - ctx->prebuf_start_ms
                                     : 0,
                                 span, ready);
                    }
                }
            }
        }

        pthread_mutex_unlock(&ctx->ab_mutex);

        if (start) {
            LOG_INFO("[%p]: PREBUFFER READY: %u ms, W:%hu R:%hu, starting playback",
                     ctx,
                     (unsigned)((ctx->ab_write - ctx->ab_read + 1) *
                                ctx->frame_size * 1000 / RAOP_SAMPLE_RATE),
                     ctx->ab_write, ctx->ab_read);
            ctx->cmd_cb(RAOP_INT_PLAY, playtime);
            ctx->prebuf_waiting = false;
        } else {
            if (!ctx->prebuf_waiting) {
                ctx->prebuf_waiting = true;
                ctx->prebuf_start_ms = gettime_ms();
            }

            request_missing(ctx);
            return;
        }
    }

    while (pushed < RTP_PUSH_MAX_FRAMES) {
        bool deliver = false;
        bool silence = false;
        u16_t len = 0;
        u32_t playtime = 0;
        u32_t now = 0;
        u16_t late;

        pthread_mutex_lock(&ctx->ab_mutex);

        if (ctx->state != RTP_PLAY || ctx->ab_read == ctx->ab_write) {
            pthread_mutex_unlock(&ctx->ab_mutex);
            break;
        }

        abuf_t *f = ctx->audio_buffer + BUFIDX(ctx->ab_read);
        now = gettime_ms();
        playtime = ctx->synchro.time +
            ((f->rtptime - ctx->synchro.rtp) * 10) /
            (RAOP_SAMPLE_RATE / 100);

        if (f->ready) {
            u32_t delta = (playtime > now) ? (playtime - now) : 0;
            late = (u16_t)((now > playtime) ? (now - playtime) : 0);

            if (late > RTP_LATE_TOLERANCE_MS) {
                f->ready = 0;
                ctx->discarded++;
                ctx->ab_read++;
                pthread_mutex_unlock(&ctx->ab_mutex);
                continue;
            }
            if (delta > hold) {
                pthread_mutex_unlock(&ctx->ab_mutex);
                break;
            }

            len = f->len;
            memcpy(ctx->playback_pcm, f->data, len);
            f->ready = 0;
            f->missed = 0;
            ctx->ab_read++;
            deliver = true;
        } else {
            u16_t ahead = (u16_t)(ctx->ab_write - ctx->ab_read);

            if (playtime > now && playtime - now > MISSING_GRACE) {
                pthread_mutex_unlock(&ctx->ab_mutex);
                break;
            }
            if (ahead == 0) {
                pthread_mutex_unlock(&ctx->ab_mutex);
                break;
            }

            len = (u16_t)(ctx->frame_size * 4);
            memset(ctx->playback_pcm, 0, len);
            f->missed = 1;
            ctx->silent_frames++;
            ctx->discarded++;
            ctx->ab_read++;
            silence = true;
            deliver = true;
        }

        pthread_mutex_unlock(&ctx->ab_mutex);

        if (deliver) {
            ctx->data_cb(ctx->playback_pcm, len,
                         silence ? playtime :
                         ((playtime > now) ? playtime : now));
            pushed++;
        }
    }

    request_missing(ctx);
}


/*---------------------------------------------------------------------------*/
#ifdef WIN32
static void *rtp_thread_func(void *arg) {
#else
static void rtp_thread_func(void *arg) {
#endif
    fd_set fds;
    int i, sock = -1;
    int count = 0;
    bool ntp_sent = false;
    char *packet = malloc(MAX_PACKET);
    rtp_t *ctx = (rtp_t *)arg;

    if (!packet)
        goto thread_exit;

    for (i = 0; i < 3; i++)
        if (ctx->rtp_sockets[i].sock > sock)
            sock = ctx->rtp_sockets[i].sock;

    for (i = 0; i < 3; i++)
        if (rtp_request_timing(ctx))
            ntp_sent = true;

    while (ctx->running) {
        struct timeval timeout = {0, 20 * 1000};
        bool any_ready = false;

        FD_ZERO(&fds);
        for (i = 0; i < 3; i++)
            FD_SET(ctx->rtp_sockets[i].sock, &fds);

        if (select(sock + 1, &fds, NULL, NULL, &timeout) <= 0) {
            if (ctx->stalled++ == 30 * 10)
                ctx->cmd_cb(RAOP_INT_STALLED);
            buffer_push_packet(ctx);
            continue;
        }

        ctx->stalled = 0;
        int order[3] = { DATA, CONTROL, TIMING };

        for (int oi = 0; oi < 3; oi++) {
            int idx = order[oi];
            if (!FD_ISSET(ctx->rtp_sockets[idx].sock, &fds))
                continue;
            any_ready = true;
            int drain_max = (idx == DATA) ? RTP_RX_DRAIN_MAX : 2;

            for (int drained = 0; drained < drain_max && ctx->running; drained++) {
                ssize_t plen;
                socklen_t rtp_client_len = sizeof(struct sockaddr_in);
                char *pktp = packet;

                plen = recvfrom(ctx->rtp_sockets[idx].sock, packet, MAX_PACKET,
                                MSG_DONTWAIT,
                                (struct sockaddr *)&ctx->rtp_host,
                                &rtp_client_len);
                if (plen <= 0)
                    break;

                if (!ntp_sent)
                    ntp_sent = rtp_request_timing(ctx);

                switch (packet[1] & ~0x80) {
                    case 0x56: {
                        seq_t seqno;
                        unsigned rtptime;
                        pktp += 4;
                        plen -= 4;
                        if (plen < 12) break;
                        seqno = ntohs(*(u16_t *)(pktp + 2));
                        rtptime = ntohl(*(u32_t *)(pktp + 4));
                        pktp += 12;
                        plen -= 12;
                        if (plen >= 16)
                            buffer_put_packet(ctx, seqno, rtptime, false, pktp, (int)plen, false);
                        break;
                    }

                    case 0x60: {
                        seq_t seqno;
                        unsigned rtptime;
                        if (plen < 12) break;
                        seqno = ntohs(*(u16_t *)(pktp + 2));
                        rtptime = ntohl(*(u32_t *)(pktp + 4));
                        pktp += 12;
                        plen -= 12;
                        if (plen < 16) break;
                        buffer_put_packet(ctx, seqno, rtptime,
                                          (packet[1] & 0x80) != 0,
                                          pktp, (int)plen, false);
                        break;
                    }

                    case 0x54: {
                        if (plen < 20) break;
                        u32_t rtp_now_latency = ntohl(*(u32_t *)(pktp + 4));
                        u64_t remote = (((u64_t)ntohl(*(u32_t *)(pktp + 8))) << 32) +
                                       ntohl(*(u32_t *)(pktp + 12));
                        u32_t rtp_now = ntohl(*(u32_t *)(pktp + 16));
                        u16_t flags = ntohs(*(u16_t *)(pktp + 2));
                        u32_t remote_gap = NTP2MS(remote - ctx->timing.remote);

                        if (!count-- || !(ctx->synchro.status & NTP_SYNC)) {
                            rtp_request_timing(ctx);
                            count = 3;
                        }
                        if (remote_gap > 10000)
                            break;

                        pthread_mutex_lock(&ctx->ab_mutex);
                        ctx->latency = rtp_now - rtp_now_latency;
                        if (flags == 7 || flags == 4)
                            ctx->latency += 11025;
                        if (ctx->latency < MIN_LATENCY)
                            ctx->latency = MIN_LATENCY;
                        else if (ctx->latency > MAX_LATENCY)
                            ctx->latency = MAX_LATENCY;
                        ctx->synchro.rtp = rtp_now - ctx->latency;
                        ctx->synchro.time = ctx->timing.local + remote_gap;
                        ctx->synchro.status |= RTP_SYNC;
                        pthread_mutex_unlock(&ctx->ab_mutex);

                        if ((ctx->synchro.status & (RTP_SYNC | NTP_SYNC)) ==
                            (RTP_SYNC | NTP_SYNC))
                            ctx->cmd_cb(RAOP_INT_TIMING);
                        break;
                    }

                    case 0x53: {
                        if (plen < 28) break;
                        u32_t reference = ntohl(*(u32_t *)(pktp + 12));
                        u64_t remote = (((u64_t)ntohl(*(u32_t *)(pktp + 16))) << 32) +
                                       ntohl(*(u32_t *)(pktp + 20));
                        u32_t roundtrip = gettime_ms() - reference;
                        if (roundtrip > 100) {
                            if (!(ctx->synchro.status & NTP_SYNC))
                                rtp_request_timing(ctx);
                            break;
                        }
                        ctx->timing.remote = remote;
                        ctx->timing.local = reference;
                        ctx->synchro.status |= NTP_SYNC;
                        break;
                    }

                    default:
                        break;
                }
            }
        }

        if (any_ready)
            buffer_push_packet(ctx);
    }

    free(packet);
thread_exit:
    LOG_INFO("[%p]: terminating", ctx);
#ifndef WIN32
    xTaskNotifyGive(ctx->joiner);
    vTaskSuspend(NULL);
#else
    return NULL;
#endif
}

/*---------------------------------------------------------------------------*/
static bool rtp_request_timing(rtp_t *ctx) {
	unsigned char req[32];
	u32_t now = gettime_ms();
	int i;
	struct sockaddr_in host;

	LOG_DEBUG("[%p]: timing request now:%u (port: %hu)", ctx, now, ctx->rtp_sockets[TIMING].rport);

	req[0] = 0x80;
	req[1] = 0x52|0x80;
	*(u16_t*)(req+2) = htons(7);
	*(u32_t*)(req+4) = htonl(0);  // dummy
	for (i = 0; i < 16; i++) req[i+8] = 0;
	*(u32_t*)(req+24) = 0;
	*(u32_t*)(req+28) = htonl(now); // this is not a real NTP, but a 32 ms counter in the low part of the NTP

	if (ctx->host.s_addr != INADDR_ANY) {
		host.sin_family = AF_INET;
		host.sin_addr =	ctx->host;
	} else host = ctx->rtp_host;

	// no address from sender, need to wait for 1st packet to be received
	if (host.sin_addr.s_addr == INADDR_ANY) return false;

	host.sin_port = htons(ctx->rtp_sockets[TIMING].rport);

	if (sizeof(req) != sendto(ctx->rtp_sockets[TIMING].sock, req, sizeof(req), MSG_DONTWAIT, (struct sockaddr*) &host, sizeof(host))) {
		LOG_WARN("[%p]: SENDTO failed (%s)", ctx, strerror(errno));
	}

	return true;
}

/*---------------------------------------------------------------------------*/
static bool rtp_request_resend(rtp_t *ctx, seq_t first, seq_t last)
{
    unsigned char req[8];
    u32_t now = gettime_ms();
    u16_t count;

    if (seq_order(last, first))
        return false;
    count = (u16_t)(last - first + 1);
    if (count == 0 || count > RTP_RESEND_MAX_RANGE)
        return false;
    if (ctx->last_resend_request && now - ctx->last_resend_request < 20)
        return false;

    req[0] = 0x80;
    req[1] = 0x55 | 0x80;
    *(u16_t *)(req + 2) = htons(1);
    *(u16_t *)(req + 4) = htons(first);
    *(u16_t *)(req + 6) = htons(count);

    ctx->rtp_host.sin_port = htons(ctx->rtp_sockets[CONTROL].rport);
    if (sizeof(req) != sendto(ctx->rtp_sockets[CONTROL].sock,
                              req, sizeof(req), MSG_DONTWAIT,
                              (struct sockaddr *)&ctx->rtp_host,
                              sizeof(ctx->rtp_host))) {
        ctx->last_resend_request = now;
        return false;
    }

    ctx->resent_req += count;
    ctx->last_resend_request = now;
    return true;
}
