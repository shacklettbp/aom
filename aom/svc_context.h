/*
 * Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

/**
 * SvcContext - input parameters and state to encode a multi-layered
 * spatial SVC frame
 */

#ifndef AOM_SVC_CONTEXT_H_
#define AOM_SVC_CONTEXT_H_

#include "./aomcx.h"
#include "./aom_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum SVC_LOG_LEVEL {
  SVC_LOG_ERROR,
  SVC_LOG_INFO,
  SVC_LOG_DEBUG
} SVC_LOG_LEVEL;

typedef struct {
  // public interface to svc_command options
  int spatial_layers;   // number of spatial layers
  int temporal_layers;  // number of temporal layers
  int temporal_layering_mode;
  SVC_LOG_LEVEL log_level;  // amount of information to display
  int log_print;       // when set, printf log messages instead of returning the
                       // message with svc_get_message
  int output_rc_stat;  // for outputting rc stats
  int speed;           // speed setting for codec
  int threads;
  int aqmode;  // turns on aq-mode=3 (cyclic_refresh): 0=off, 1=on.
  // private storage for aom_svc_encode
  void *internal;
} SvcContext;

#define OPTION_BUFFER_SIZE 1024
#define COMPONENTS 4  // psnr & sse statistics maintained for total, y, u, v

typedef struct SvcInternal {
  char options[OPTION_BUFFER_SIZE];  // set by aom_svc_set_options

  // values extracted from option, quantizers
  aom_svc_extra_cfg_t svc_params;
  int enable_auto_alt_ref[AOM_SS_MAX_LAYERS];
  int bitrates[AOM_SS_MAX_LAYERS];

  // accumulated statistics
  double psnr_sum[AOM_SS_MAX_LAYERS][COMPONENTS];  // total/Y/U/V
  uint64_t sse_sum[AOM_SS_MAX_LAYERS][COMPONENTS];
  uint32_t bytes_sum[AOM_SS_MAX_LAYERS];

  // codec encoding values
  int width;    // width of highest layer
  int height;   // height of highest layer
  int kf_dist;  // distance between keyframes

  // state variables
  int psnr_pkt_received;
  int layer;
  int use_multiple_frame_contexts;

  char message_buffer[2048];
  aom_codec_ctx_t *codec_ctx;
} SvcInternal_t;

/**
 * Set SVC options
 * options are supplied as a single string separated by spaces
 * Format: encoding-mode=<i|ip|alt-ip|gf>
 *         layers=<layer_count>
 *         scaling-factors=<n1>/<d1>,<n2>/<d2>,...
 *         quantizers=<q1>,<q2>,...
 */
aom_codec_err_t aom_svc_set_options(SvcContext *svc_ctx, const char *options);

/**
 * initialize SVC encoding
 */
aom_codec_err_t aom_svc_init(SvcContext *svc_ctx, aom_codec_ctx_t *codec_ctx,
                             aom_codec_iface_t *iface,
                             aom_codec_enc_cfg_t *cfg);
/**
 * encode a frame of video with multiple layers
 */
aom_codec_err_t aom_svc_encode(SvcContext *svc_ctx, aom_codec_ctx_t *codec_ctx,
                               struct aom_image *rawimg, aom_codec_pts_t pts,
                               int64_t duration, int deadline);

/**
 * finished with svc encoding, release allocated resources
 */
void aom_svc_release(SvcContext *svc_ctx);

/**
 * dump accumulated statistics and reset accumulated values
 */
const char *aom_svc_dump_statistics(SvcContext *svc_ctx);

/**
 *  get status message from previous encode
 */
const char *aom_svc_get_message(const SvcContext *svc_ctx);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AOM_SVC_CONTEXT_H_
