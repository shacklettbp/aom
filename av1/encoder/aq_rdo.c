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
#include "aom_ports/system_state.h"

#include "av1/encoder/aq_variance.h"

#include "aom_ports/system_state.h"
#include "av1/common/seg_common.h"
#include "av1/encoder/ratectrl.h"
#include "av1/encoder/rd.h"
#include "av1/encoder/rdopt.h"
#include "av1/encoder/segmentation.h"

DECLARE_ALIGNED(16, static const uint8_t, av1_64_zeros[64]) = { 0 };

static const double rate_ratio[MAX_SEGMENTS] = { 0.25, 0.5, 0.75, 1.0,
                                                 1.25, 1.5, 2.0,  2.5 };

void av1_rdo_aq_frame_setup(AV1_COMP *cpi) {
  AV1_COMMON *cm = &cpi->common;
  struct segmentation *seg = &cm->seg;
  int i;

  av1_enable_segmentation(seg);
  av1_clearall_segfeatures(seg);

  seg->abs_delta = SEGMENT_DELTADATA;

  aom_clear_system_state();

  for (i = 0; i < MAX_SEGMENTS; ++i) {
    int qindex_delta;

    // No need to enable SEG_LVL_ALT_Q for this segment.
    if (rate_ratio[i] == 1.0) {
      continue;
    }

    qindex_delta =
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
int av1_rdo_aq_select_segment(AV1_COMP *cpi, MACROBLOCK *mb, BLOCK_SIZE bs, int mi_row, int mi_col) {
  int cur_segment, best_segment;
  int64_t rd_min;
  MACROBLOCKD *xd;
  AV1_COMMON *cm = &cpi->common;
  struct seg_counts counts[MAX_SEGMENTS];

  aom_clear_system_state();

  xd = &mb->e_mbd;
  rd_min = INT64_MAX;
  best_segment = -1;

  for (cur_segment = 0; cur_segment < MAX_SEGMENTS; cur_segment++) {
    int mb_rate, seg_rate, approx_rate, qstep;
    int64_t mb_distortion, rd;
    unsigned int sse;

    memcpy(&counts[cur_segment], &cm->counts.seg, sizeof(struct seg_counts));

    qstep = get_segdata(&cm->seg, cur_segment, SEG_LVL_ALT_Q) + cm->base_qindex;
    qstep /= 8;
    cpi->fn_ptr[bs].vf(mb->plane[0].src.buf, mb->plane[0].src.stride, av1_64_zeros, 0, &sse);
    av1_model_rd_from_var_lapndz(sse, num_pels_log2_lookup[bs], qstep, &mb_rate, &mb_distortion);

    seg_rate = av1_calc_segmap_cost(cm, xd, cur_segment, bs, mi_row, mi_col, &counts[cur_segment]);

    approx_rate = seg_rate + mb_rate;

    rd = RDCOST(mb->rdmult, mb->rddiv, approx_rate, mb_distortion);
    if (rd < rd_min) {
      rd_min = rd;
      best_segment = cur_segment;
    }
  }

  memcpy(&cm->counts.seg, &counts[best_segment], sizeof(struct seg_counts));

  return best_segment;
}
