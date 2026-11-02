#ifndef DSP_API_H
#define DSP_API_H

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register the /dsp UI page and the /api/dsp/ JSON endpoints on an
 *        already-running httpd server. Call this after web_server_start().
 *        Does not modify or depend on any internals of web_server.c.
 */
esp_err_t dsp_api_register(httpd_handle_t server);

#ifdef __cplusplus
}
#endif

#endif // DSP_API_H
