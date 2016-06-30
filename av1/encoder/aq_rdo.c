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

#include "av1/encoder/aq_rdo.h"
#include "av1/encoder/aq_variance.h"

#include "aom_ports/system_state.h"
#include "av1/common/pred_common.h"
#include "av1/common/seg_common.h"
#include "av1/encoder/ratectrl.h"
#include "av1/encoder/rd.h"
#include "av1/encoder/rdopt.h"
#include "av1/encoder/segmentation.h"

static const double rate_ratio[MAX_SEGMENTS] = { 0.25, 0.5, 0.75, 1.0,
                                                 1.25, 1.5, 1.75,  2.0 };

DECLARE_ALIGNED(16, static const uint8_t, av1_64_zeros[64]) = { 0 };
#if CONFIG_AOM_HIGHBITDEPTH
DECLARE_ALIGNED(16, static const uint16_t, av1_highbd_64_zeros[64]) = { 0 };
#endif

void av1_rdo_aq_frame_setup(AV1_COMP *cpi) {
  AV1_COMMON *cm = &cpi->common;
  struct segmentation *seg = &cm->seg;
  int i;

  av1_enable_segmentation(seg);
  av1_clearall_segfeatures(seg);

  seg->abs_delta = SEGMENT_DELTADATA;

  aom_clear_system_state();

  for (i = 0; i < MAX_SEGMENTS; i++) {
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

int av1_rdo_aq_seg_rate(AV1_COMMON *cm, MACROBLOCKD *xd, int segment_id, BLOCK_SIZE bs)
{ 
#if CONFIG_MISC_FIXES
  struct segmentation_probs *segp = &cm->fc->seg;
#else
  struct segmentation_probs *segp = &cm->segp;
#endif

  int mi_row, mi_col;
  aom_prob *probs;

  mi_row = -xd->mb_to_top_edge / 8 / MI_SIZE;
  mi_col = -xd->mb_to_left_edge / 8 / MI_SIZE;

  if (frame_is_intra_only(cm) || cm->error_resilient_mode) {
    probs = segp->tree_probs;
  }
  else {
    const int pred_segment_id = get_segment_id(cm, cm->last_frame_seg_map, bs, mi_row, mi_col);
    if (pred_segment_id == segment_id)
      return 0;

    probs = segp->pred_probs;
  }
  if (segment_id & 1) {
    return av1_cost_one(probs[3 + (segment_id >> 2)]);
  } else {
    return av1_cost_zero(probs[3 + (segment_id >> 2)]);
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

  //if (right_overflow || bottom_overflow) {
  //  const int bw = 8 * num_8x8_blocks_wide_lookup[bs] - right_overflow;
  //  const int bh = 8 * num_8x8_blocks_high_lookup[bs] - bottom_overflow;
  //} else {
  //  var = cpi->fn_ptr[bs].vf(x->plane[0].src.buf, x->plane[0].src.stride, zeros, 0, &sse);
  //}

  return (256 * (uint64_t)cpi->fn_ptr[bs].vf(p->src.buf, p->src.stride, zeros, 0, &sse)) >> num_pels_log2_lookup[bs];
}

static unsigned total_variance(AV1_COMP *cpi, MACROBLOCK *x, BLOCK_SIZE bs) {
  int i;
  unsigned int total_var = 0;
  for (i = 0; i < MAX_MB_PLANE; i++)
    total_var += block_variance(cpi, x, bs, i);

  return total_var;
}

int av1_rdo_aq_dist_scale(AV1_COMP *cpi, MACROBLOCK *x, BLOCK_SIZE bs) {
  unsigned int var = total_variance(cpi, x, bs);
  double scale;
  aom_clear_system_state();

  scale = 0.176782*pow(var, 0.173283);
  //scale = 5.65669*pow(var, -0.173283);

  //printf("scale: %f %u\n", scale, var);
  //printf("var: %u scale: %f ", var, scale);

  //return x->rd_dist_scale;
  return round((double)x->rd_dist_scale * scale);
}

#if 0
/* Perform RDO on the different segment possibilities to choose a segment */
int av1_rdo_aq_select_segment(AV1_COMP *cpi, MACROBLOCK *mb, BLOCK_SIZE bs) {
  unsigned int sse, var;
  int cur_segment, best_segment;
  int64_t rd_min;
  MACROBLOCKD *xd;
  AV1_COMMON *cm = &cpi->common;

  aom_clear_system_state();

  xd = &mb->e_mbd;
  rd_min = INT64_MAX;
  best_segment = -1;

  //cpi->fn_ptr[bs].vf(mb->plane[0].src.buf, mb->plane[0].src.stride, xd->plane[0].dst.buf, xd->plane[0].dst.stride, &sse);
  //var = sse;
  //var = (var * 256) >> num_pels_log2_lookup[bs];
  //var = mb->source_variance;

  for (cur_segment = 0; cur_segment < MAX_SEGMENTS; cur_segment++) {
    int mb_rate, seg_rate, approx_rate, qstep, quant;
    int64_t mb_distortion, rd;

    //quant = cpi->y_dequant[get_segdata(&cm->seg, cur_segment, SEG_LVL_ALT_Q) + cm->base_qindex][1];
    //qstep = quant >> 3;

    //av1_model_rd_from_var_lapndz(var, num_pels_log2_lookup[bs], qstep, &mb_rate, &mb_distortion);
    //mb_distortion <<= 10;
    
    seg_rate = av1_rdo_aq_seg_rate(cm, xd, cur_segment, bs);

    approx_rate = seg_rate + mb_rate;

    //if (approx_rate > rate_limit)
    //  continue;

    rd = RDCOST(mb->rdmult, mb->rd_dist_scale, approx_rate, mb_distortion);
    printf("aq: %d %d %d %d %d %d %d\n", var, num_pels_log2_lookup[bs], qstep, seg_rate, mb_rate, mb_distortion, rd);
    if (rd < rd_min) {
      rd_min = rd;
      best_segment = cur_segment;
    }
  }

  if (best_segment == -1)
    best_segment = 0;

  printf("%d\n", best_segment);

  return best_segment;
}
#endif
