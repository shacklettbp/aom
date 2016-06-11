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

#include <math.h>

#include "aom_ports/mem.h"

#include "av1/encoder/aq_variance.h"

#include "av1/common/seg_common.h"
#include "av1/encoder/ratectrl.h"
#include "av1/encoder/rd.h"
#include "av1/encoder/segmentation.h"
#include "aom_ports/system_state.h"

static const double rate_ratio[MAX_SEGMENTS] = { 3.0, 2.5,  2.0, 1.5, 1.25, 1.0,
                                                 0.75, 0.5, 0.25 };

void av1_aq_frame_setup(AV1_COMP *cpi) {
  AV1_COMMON *cm = &cpi->common;
  struct segmentation *seg = &cm->seg;
  int i;

  av1_enable_segmentation(seg);
  av1_clearall_segfeatures(seg);

  seg->abs_delta = SEGMENT_DELTADATA;

  aom_clear_system_state();

  for (i = 0; i < MAX_SEGMENTS; ++i) {
    // No need to enable SEG_LVL_ALT_Q for this segment.
    if (rate_ratio[i] == 1.0) {
      continue;
    }

    int qindex_delta =
        av1_compute_qdelta_by_rate(&cpi->rc, cm->frame_type, cm->base_qindex,
                                   rate_ratio[i], cm->bit_depth);

    // We don't allow qindex 0 in a segment if the base value is not 0.
    // Q index 0 (lossless) implies 4x4 encoding only and in AQ mode a segment
    // Q delta is sometimes applied without going back around the rd loop.
    // This could lead to an illegal combination of partition size and q.
    if ((cm->base_qindex != 0) && ((cm->base_qindex + qindex_delta) == 0)) {
      qindex_delta = -cm->base_qindex + 1;
    }

    av1_set_segdata(seg, i, SEG_LVL_ALT_Q, qindex_delta);
    av1_enable_segfeature(seg, i, SEG_LVL_ALT_Q);
  }
  
}

/* Perform RDO on the different segment possibilities to choose a segment */
void av1_aq_select_segment(AV1_COMP *cpi, MACROBLOCK *mb, BLOCK_SIZE bs,
                               int mi_row, int mi_col, int projected_rate) {
  AV1_COMMON *const cm = &cpi->common;
  AQ_DISTORTION_MODE distortion_mode = cpi->oxcf.aq_distortion_mode;
  int x, y, i;

  aom_clear_system_state();

  double lambda = 40;
  double rd_min = INFINITY;

  int segment_min = 0;

  for (i = 0; i < MAX_SEGMENTS; i++) {

  }
}
