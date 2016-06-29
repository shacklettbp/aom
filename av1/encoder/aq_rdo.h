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

#ifndef AV1_ENCODER_AQ_RDO_H_
#define AV1_ENCODER_AQ_RDO_H_

#include "av1/encoder/encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

void av1_rdo_aq_frame_setup(AV1_COMP *cpi);
int av1_rdo_aq_seg_rate(AV1_COMMON *cm, MACROBLOCKD *xd, int segment_id, BLOCK_SIZE bs);
int av1_rdo_aq_dist_scale(AV1_COMP *cpi, MACROBLOCK *x, BLOCK_SIZE bs);
int av1_rdo_aq_select_segment(AV1_COMP *cpi, MACROBLOCK *mb, BLOCK_SIZE bs);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AV1_ENCODER_AQ_VARIANCE_H_
