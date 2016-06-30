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

static const double q_ratio[MAX_SEGMENTS] = { 0.5, 0.65, 0.80, 1.0,
                                              1.25, 1.5, 2.0, 2.5 };

DECLARE_ALIGNED(16, static const uint8_t, av1_64_zeros[64]) = { 0 };
#if CONFIG_AOM_HIGHBITDEPTH
DECLARE_ALIGNED(16, static const uint16_t, av1_highbd_64_zeros[64]) = { 0 };
#endif

void av1_vaq_frame_setup(AV1_COMP *cpi) {
  AV1_COMMON *cm = &cpi->common;
  struct segmentation *seg = &cm->seg;
  int i;

  if (frame_is_intra_only(cm) || cm->error_resilient_mode ||
      cpi->refresh_alt_ref_frame ||
      (cpi->refresh_golden_frame && !cpi->rc.is_src_frame_alt_ref)) {
    av1_enable_segmentation(seg);
    av1_clearall_segfeatures(seg);

    seg->abs_delta = SEGMENT_DELTADATA;

    aom_clear_system_state();

    for (i = 0; i < MAX_SEGMENTS; ++i) {
      int base_q;
      int qindex_delta;

      // No need to enable SEG_LVL_ALT_Q for this segment.
      if (q_ratio[i] == 1.0) {
        continue;
      }

      /* Calculate delta */
      base_q = av1_convert_qindex_to_q(cm->base_qindex, cm->bit_depth);
      qindex_delta = av1_compute_qdelta(&cpi->rc, base_q, base_q * q_ratio[i], cm->bit_depth);

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
}

static unsigned block_variance(AV1_COMP *cpi, MACROBLOCK *x, BLOCK_SIZE bsize, int plane_idx) {
  unsigned int var, sse;

  MACROBLOCKD *xd = &x->e_mbd;
  struct macroblock_plane *const p = &x->plane[plane_idx];
  struct macroblockd_plane *const pd = &xd->plane[plane_idx];
  BLOCK_SIZE bs = get_plane_block_size(bsize, pd);
  int right_overflow, bottom_overflow;

  const uint8_t *zeros =
#if CONFIG_AOM_HIGHBITDEPTH
    ((xd->cur_buf->flags & YV12_FLAG_HIGHBITDEPTH) ? CONVERT_TO_BYTEPTR(av1_highbd_64_zeros) : av1_64_zeros);
#else
    av1_64_zeros;
#endif
  
  if (bs == BLOCK_INVALID)
    return 0; // FIXME this happens with 4x4, what's the "correct" way

  right_overflow =
      (xd->mb_to_right_edge < 0) ? ((-xd->mb_to_right_edge) >> 3) : 0;
  bottom_overflow =
      (xd->mb_to_bottom_edge < 0) ? ((-xd->mb_to_bottom_edge) >> 3) : 0;

  //assert(!right_overflow && !bottom_overflow);

  //if (right_overflow || bottom_overflow) {
  //  const int bw = 8 * num_8x8_blocks_wide_lookup[bs] - right_overflow;
  //  const int bh = 8 * num_8x8_blocks_high_lookup[bs] - bottom_overflow;
  //} else {
  //  var = cpi->fn_ptr[bs].vf(x->plane[0].src.buf, x->plane[0].src.stride, zeros, 0, &sse);
  //}

  //return (uint64_t)cpi->fn_ptr[bs].vf(p->src.buf, p->src.stride, zeros, 0, &sse);
  return (256 * (uint64_t)cpi->fn_ptr[bs].vf(p->src.buf, p->src.stride, zeros, 0, &sse)) >> num_pels_log2_lookup[bs];
}

static unsigned total_variance(AV1_COMP *cpi, MACROBLOCK *x, BLOCK_SIZE bs) {
  int i;
  unsigned int total_var = 0;
  for (i = 0; i < MAX_MB_PLANE; i++)
    total_var += block_variance(cpi, x, bs, i);

  return total_var;
}

unsigned int av1_vaq_segment_id(AV1_COMP *cpi, MACROBLOCK *x, BLOCK_SIZE bs) {
  int i;
  unsigned int var, best_segment;
  double ideal_ratio, min_delta, delta;
  aom_clear_system_state();

  var = total_variance(cpi, x, bs);
  //if (cpi->oxcf.pass == 2) {
  //  double midpoint = cpi->twopass.mb_av_energy;
  //  ideal_ratio = exp(-0.120111*midpoint)*pow(var, 0.173283);
  //} else {
  //  ideal_ratio = 0.176782*pow(var, 0.173283);
  //}
  ideal_ratio = 0.176782*pow(var, 2*0.173283);

  min_delta = INFINITY;
  for (i = 0; i < MAX_SEGMENTS; i++) {
    delta = fabs(q_ratio[i] - ideal_ratio);

    if (delta < min_delta) {
      best_segment = i;
      min_delta = delta;
    }
  }

  printf("var: %u, ratio: %f, segment: %u\n", var, ideal_ratio, best_segment);

  return best_segment;
}
