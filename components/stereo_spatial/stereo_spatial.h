#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STEREO_SPATIAL_MODE_OFF = 0,
    STEREO_SPATIAL_MODE_NATURAL,
    STEREO_SPATIAL_MODE_WIDE
} stereo_spatial_mode_t;

typedef struct {
    stereo_spatial_mode_t mode;
    bool enable_width_eq;
    bool enable_decorrelator;
} stereo_spatial_config_t;

typedef struct stereo_spatial_ctx stereo_spatial_ctx_t;

stereo_spatial_ctx_t* stereo_spatial_init(uint32_t sample_rate);
void stereo_spatial_free(stereo_spatial_ctx_t *ctx);

void stereo_spatial_get_config(const stereo_spatial_ctx_t *ctx, stereo_spatial_config_t *cfg);
void stereo_spatial_set_config(stereo_spatial_ctx_t *ctx, const stereo_spatial_config_t *cfg);

void stereo_spatial_process_i16(stereo_spatial_ctx_t *ctx, int16_t *buffer, size_t frames);

#ifdef __cplusplus
}
#endif