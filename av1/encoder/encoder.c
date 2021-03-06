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

#include <limits.h>
#include <math.h>
#include <stdio.h>

#include "./aom_config.h"

#include "av1/common/alloccommon.h"
#if CONFIG_CLPF
#include "av1/common/clpf.h"
#endif
#if CONFIG_DERING
#include "av1/common/dering.h"
#endif  // CONFIG_DERING
#include "av1/common/filter.h"
#include "av1/common/idct.h"
#include "av1/common/reconinter.h"
#include "av1/common/reconintra.h"
#include "av1/common/tile_common.h"

#include "av1/encoder/aq_complexity.h"
#include "av1/encoder/aq_cyclicrefresh.h"
#include "av1/encoder/aq_variance.h"
#include "av1/encoder/bitstream.h"
#include "av1/encoder/context_tree.h"
#include "av1/encoder/encodeframe.h"
#include "av1/encoder/encodemv.h"
#include "av1/encoder/encoder.h"
#include "av1/encoder/ethread.h"
#include "av1/encoder/firstpass.h"
#include "av1/encoder/mbgraph.h"
#include "av1/encoder/picklpf.h"
#include "av1/encoder/ratectrl.h"
#include "av1/encoder/rd.h"
#include "av1/encoder/resize.h"
#include "av1/encoder/segmentation.h"
#include "av1/encoder/skin_detection.h"
#include "av1/encoder/speed_features.h"
#include "av1/encoder/temporal_filter.h"

#include "./aom_dsp_rtcd.h"
#include "./aom_scale_rtcd.h"
#include "./av1_rtcd.h"
#include "aom_dsp/psnr.h"
#if CONFIG_INTERNAL_STATS
#include "aom_dsp/ssim.h"
#endif
#include "aom_dsp/aom_dsp_common.h"
#include "aom_dsp/aom_filter.h"
#include "aom_ports/aom_timer.h"
#include "aom_ports/mem.h"
#include "aom_ports/system_state.h"
#include "aom_scale/aom_scale.h"

#define AM_SEGMENT_ID_INACTIVE 7
#define AM_SEGMENT_ID_ACTIVE 0

#define SHARP_FILTER_QTHRESH 0 /* Q threshold for 8-tap sharp filter */

#define ALTREF_HIGH_PRECISION_MV 1     // Whether to use high precision mv
                                       //  for altref computation.
#define HIGH_PRECISION_MV_QTHRESH 200  // Q threshold for high precision
                                       // mv. Choose a very high value for
                                       // now so that HIGH_PRECISION is always
                                       // chosen.
// #define OUTPUT_YUV_REC

#ifdef OUTPUT_YUV_DENOISED
FILE *yuv_denoised_file = NULL;
#endif
#ifdef OUTPUT_YUV_SKINMAP
FILE *yuv_skinmap_file = NULL;
#endif
#ifdef OUTPUT_YUV_REC
FILE *yuv_rec_file;
#endif

#if 0
FILE *framepsnr;
FILE *kf_list;
FILE *keyfile;
#endif

static INLINE void Scale2Ratio(AOM_SCALING mode, int *hr, int *hs) {
  switch (mode) {
    case NORMAL:
      *hr = 1;
      *hs = 1;
      break;
    case FOURFIVE:
      *hr = 4;
      *hs = 5;
      break;
    case THREEFIVE:
      *hr = 3;
      *hs = 5;
      break;
    case ONETWO:
      *hr = 1;
      *hs = 2;
      break;
    default:
      *hr = 1;
      *hs = 1;
      assert(0);
      break;
  }
}

// Mark all inactive blocks as active. Other segmentation features may be set
// so memset cannot be used, instead only inactive blocks should be reset.
static void suppress_active_map(AV1_COMP *cpi) {
  unsigned char *const seg_map = cpi->segmentation_map;
  int i;
  if (cpi->active_map.enabled || cpi->active_map.update)
    for (i = 0; i < cpi->common.mi_rows * cpi->common.mi_cols; ++i)
      if (seg_map[i] == AM_SEGMENT_ID_INACTIVE)
        seg_map[i] = AM_SEGMENT_ID_ACTIVE;
}

static void apply_active_map(AV1_COMP *cpi) {
  struct segmentation *const seg = &cpi->common.seg;
  unsigned char *const seg_map = cpi->segmentation_map;
  const unsigned char *const active_map = cpi->active_map.map;
  int i;

  assert(AM_SEGMENT_ID_ACTIVE == CR_SEGMENT_ID_BASE);

  if (frame_is_intra_only(&cpi->common)) {
    cpi->active_map.enabled = 0;
    cpi->active_map.update = 1;
  }

  if (cpi->active_map.update) {
    if (cpi->active_map.enabled) {
      for (i = 0; i < cpi->common.mi_rows * cpi->common.mi_cols; ++i)
        if (seg_map[i] == AM_SEGMENT_ID_ACTIVE) seg_map[i] = active_map[i];
      av1_enable_segmentation(seg);
      av1_enable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_SKIP);
      av1_enable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF);
      // Setting the data to -MAX_LOOP_FILTER will result in the computed loop
      // filter level being zero regardless of the value of seg->abs_delta.
      av1_set_segdata(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF,
                      -MAX_LOOP_FILTER);
    } else {
      av1_disable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_SKIP);
      av1_disable_segfeature(seg, AM_SEGMENT_ID_INACTIVE, SEG_LVL_ALT_LF);
      if (seg->enabled) {
        seg->update_data = 1;
        seg->update_map = 1;
      }
    }
    cpi->active_map.update = 0;
  }
}

int av1_set_active_map(AV1_COMP *cpi, unsigned char *new_map_16x16, int rows,
                       int cols) {
  if (rows == cpi->common.mb_rows && cols == cpi->common.mb_cols) {
    unsigned char *const active_map_8x8 = cpi->active_map.map;
    const int mi_rows = cpi->common.mi_rows;
    const int mi_cols = cpi->common.mi_cols;
    cpi->active_map.update = 1;
    if (new_map_16x16) {
      int r, c;
      for (r = 0; r < mi_rows; ++r) {
        for (c = 0; c < mi_cols; ++c) {
          active_map_8x8[r * mi_cols + c] =
              new_map_16x16[(r >> 1) * cols + (c >> 1)]
                  ? AM_SEGMENT_ID_ACTIVE
                  : AM_SEGMENT_ID_INACTIVE;
        }
      }
      cpi->active_map.enabled = 1;
    } else {
      cpi->active_map.enabled = 0;
    }
    return 0;
  } else {
    return -1;
  }
}

int av1_get_active_map(AV1_COMP *cpi, unsigned char *new_map_16x16, int rows,
                       int cols) {
  if (rows == cpi->common.mb_rows && cols == cpi->common.mb_cols &&
      new_map_16x16) {
    unsigned char *const seg_map_8x8 = cpi->segmentation_map;
    const int mi_rows = cpi->common.mi_rows;
    const int mi_cols = cpi->common.mi_cols;
    memset(new_map_16x16, !cpi->active_map.enabled, rows * cols);
    if (cpi->active_map.enabled) {
      int r, c;
      for (r = 0; r < mi_rows; ++r) {
        for (c = 0; c < mi_cols; ++c) {
          // Cyclic refresh segments are considered active despite not having
          // AM_SEGMENT_ID_ACTIVE
          new_map_16x16[(r >> 1) * cols + (c >> 1)] |=
              seg_map_8x8[r * mi_cols + c] != AM_SEGMENT_ID_INACTIVE;
        }
      }
    }
    return 0;
  } else {
    return -1;
  }
}

void av1_set_high_precision_mv(AV1_COMP *cpi, int allow_high_precision_mv) {
  MACROBLOCK *const mb = &cpi->td.mb;
  cpi->common.allow_high_precision_mv = allow_high_precision_mv;

#if CONFIG_REF_MV
  if (cpi->common.allow_high_precision_mv) {
    int i;
    for (i = 0; i < NMV_CONTEXTS; ++i) {
      mb->mv_cost_stack[i] = mb->nmvcost_hp[i];
      mb->mvsadcost = mb->nmvsadcost_hp;
    }
  } else {
    int i;
    for (i = 0; i < NMV_CONTEXTS; ++i) {
      mb->mv_cost_stack[i] = mb->nmvcost[i];
      mb->mvsadcost = mb->nmvsadcost;
    }
  }
#else
  if (cpi->common.allow_high_precision_mv) {
    mb->mvcost = mb->nmvcost_hp;
    mb->mvsadcost = mb->nmvsadcost_hp;
  } else {
    mb->mvcost = mb->nmvcost;
    mb->mvsadcost = mb->nmvsadcost;
  }
#endif
}

static void setup_frame(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  // Set up entropy context depending on frame type. The decoder mandates
  // the use of the default context, index 0, for keyframes and inter
  // frames where the error_resilient_mode or intra_only flag is set. For
  // other inter-frames the encoder currently uses only two contexts;
  // context 1 for ALTREF frames and context 0 for the others.
  if (frame_is_intra_only(cm) || cm->error_resilient_mode) {
    av1_setup_past_independence(cm);
  } else {
    cm->frame_context_idx = cpi->refresh_alt_ref_frame;
  }

  if (cm->frame_type == KEY_FRAME) {
    cpi->refresh_golden_frame = 1;
    cpi->refresh_alt_ref_frame = 1;
    av1_zero(cpi->interp_filter_selected);
  } else {
    *cm->fc = cm->frame_contexts[cm->frame_context_idx];
    av1_zero(cpi->interp_filter_selected[0]);
  }
}

static void av1_enc_setup_mi(AV1_COMMON *cm) {
  int i;
  cm->mi = cm->mip + cm->mi_stride + 1;
  memset(cm->mip, 0, cm->mi_stride * (cm->mi_rows + 1) * sizeof(*cm->mip));
  cm->prev_mi = cm->prev_mip + cm->mi_stride + 1;
  // Clear top border row
  memset(cm->prev_mip, 0, sizeof(*cm->prev_mip) * cm->mi_stride);
  // Clear left border column
  for (i = 1; i < cm->mi_rows + 1; ++i)
    memset(&cm->prev_mip[i * cm->mi_stride], 0, sizeof(*cm->prev_mip));

  cm->mi_grid_visible = cm->mi_grid_base + cm->mi_stride + 1;
  cm->prev_mi_grid_visible = cm->prev_mi_grid_base + cm->mi_stride + 1;

  memset(cm->mi_grid_base, 0,
         cm->mi_stride * (cm->mi_rows + 1) * sizeof(*cm->mi_grid_base));
}

static int av1_enc_alloc_mi(AV1_COMMON *cm, int mi_size) {
  cm->mip = aom_calloc(mi_size, sizeof(*cm->mip));
  if (!cm->mip) return 1;
  cm->prev_mip = aom_calloc(mi_size, sizeof(*cm->prev_mip));
  if (!cm->prev_mip) return 1;
  cm->mi_alloc_size = mi_size;

  cm->mi_grid_base = (MODE_INFO **)aom_calloc(mi_size, sizeof(MODE_INFO *));
  if (!cm->mi_grid_base) return 1;
  cm->prev_mi_grid_base =
      (MODE_INFO **)aom_calloc(mi_size, sizeof(MODE_INFO *));
  if (!cm->prev_mi_grid_base) return 1;

  return 0;
}

static void av1_enc_free_mi(AV1_COMMON *cm) {
  aom_free(cm->mip);
  cm->mip = NULL;
  aom_free(cm->prev_mip);
  cm->prev_mip = NULL;
  aom_free(cm->mi_grid_base);
  cm->mi_grid_base = NULL;
  aom_free(cm->prev_mi_grid_base);
  cm->prev_mi_grid_base = NULL;
}

static void av1_swap_mi_and_prev_mi(AV1_COMMON *cm) {
  // Current mip will be the prev_mip for the next frame.
  MODE_INFO **temp_base = cm->prev_mi_grid_base;
  MODE_INFO *temp = cm->prev_mip;
  cm->prev_mip = cm->mip;
  cm->mip = temp;

  // Update the upper left visible macroblock ptrs.
  cm->mi = cm->mip + cm->mi_stride + 1;
  cm->prev_mi = cm->prev_mip + cm->mi_stride + 1;

  cm->prev_mi_grid_base = cm->mi_grid_base;
  cm->mi_grid_base = temp_base;
  cm->mi_grid_visible = cm->mi_grid_base + cm->mi_stride + 1;
  cm->prev_mi_grid_visible = cm->prev_mi_grid_base + cm->mi_stride + 1;
}

void av1_initialize_enc(void) {
  static volatile int init_done = 0;

  if (!init_done) {
    av1_rtcd();
    aom_dsp_rtcd();
    aom_scale_rtcd();
    av1_init_intra_predictors();
    av1_init_me_luts();
    av1_rc_init_minq_luts();
    av1_entropy_mv_init();
    av1_encode_token_init();
    init_done = 1;
  }
}

static void dealloc_compressor_data(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
#if CONFIG_REF_MV
  int i;
#endif

  aom_free(cpi->mbmi_ext_base);
  cpi->mbmi_ext_base = NULL;

  aom_free(cpi->tile_data);
  cpi->tile_data = NULL;

  // Delete sementation map
  aom_free(cpi->segmentation_map);
  cpi->segmentation_map = NULL;
  aom_free(cpi->coding_context.last_frame_seg_map_copy);
  cpi->coding_context.last_frame_seg_map_copy = NULL;

#if CONFIG_REF_MV
  for (i = 0; i < NMV_CONTEXTS; ++i) {
    aom_free(cpi->nmv_costs[i][0]);
    aom_free(cpi->nmv_costs[i][1]);
    aom_free(cpi->nmv_costs_hp[i][0]);
    aom_free(cpi->nmv_costs_hp[i][1]);
    cpi->nmv_costs[i][0] = NULL;
    cpi->nmv_costs[i][1] = NULL;
    cpi->nmv_costs_hp[i][0] = NULL;
    cpi->nmv_costs_hp[i][1] = NULL;
  }
#endif

  aom_free(cpi->nmvcosts[0]);
  aom_free(cpi->nmvcosts[1]);
  cpi->nmvcosts[0] = NULL;
  cpi->nmvcosts[1] = NULL;

  aom_free(cpi->nmvcosts_hp[0]);
  aom_free(cpi->nmvcosts_hp[1]);
  cpi->nmvcosts_hp[0] = NULL;
  cpi->nmvcosts_hp[1] = NULL;

  aom_free(cpi->nmvsadcosts[0]);
  aom_free(cpi->nmvsadcosts[1]);
  cpi->nmvsadcosts[0] = NULL;
  cpi->nmvsadcosts[1] = NULL;

  aom_free(cpi->nmvsadcosts_hp[0]);
  aom_free(cpi->nmvsadcosts_hp[1]);
  cpi->nmvsadcosts_hp[0] = NULL;
  cpi->nmvsadcosts_hp[1] = NULL;

  av1_cyclic_refresh_free(cpi->cyclic_refresh);
  cpi->cyclic_refresh = NULL;

  aom_free(cpi->active_map.map);
  cpi->active_map.map = NULL;

  av1_free_ref_frame_buffers(cm->buffer_pool);
  av1_free_context_buffers(cm);

  aom_free_frame_buffer(&cpi->last_frame_uf);
  aom_free_frame_buffer(&cpi->scaled_source);
  aom_free_frame_buffer(&cpi->scaled_last_source);
  aom_free_frame_buffer(&cpi->alt_ref_buffer);
  av1_lookahead_destroy(cpi->lookahead);

  aom_free(cpi->tile_tok[0][0]);
  cpi->tile_tok[0][0] = 0;

  av1_free_pc_tree(&cpi->td);

  if (cpi->source_diff_var != NULL) {
    aom_free(cpi->source_diff_var);
    cpi->source_diff_var = NULL;
  }
}

static void save_coding_context(AV1_COMP *cpi) {
  CODING_CONTEXT *const cc = &cpi->coding_context;
  AV1_COMMON *cm = &cpi->common;
#if CONFIG_REF_MV
  int i;
#endif

// Stores a snapshot of key state variables which can subsequently be
// restored with a call to av1_restore_coding_context. These functions are
// intended for use in a re-code loop in av1_compress_frame where the
// quantizer value is adjusted between loop iterations.
#if CONFIG_REF_MV
  for (i = 0; i < NMV_CONTEXTS; ++i) {
    av1_copy(cc->nmv_vec_cost[i], cpi->td.mb.nmv_vec_cost[i]);
    memcpy(cc->nmv_costs[i][0], cpi->nmv_costs[i][0],
           MV_VALS * sizeof(*cpi->nmv_costs[i][0]));
    memcpy(cc->nmv_costs[i][1], cpi->nmv_costs[i][1],
           MV_VALS * sizeof(*cpi->nmv_costs[i][1]));
    memcpy(cc->nmv_costs_hp[i][0], cpi->nmv_costs_hp[i][0],
           MV_VALS * sizeof(*cpi->nmv_costs_hp[i][0]));
    memcpy(cc->nmv_costs_hp[i][1], cpi->nmv_costs_hp[i][1],
           MV_VALS * sizeof(*cpi->nmv_costs_hp[i][1]));
  }
#else
  av1_copy(cc->nmvjointcost, cpi->td.mb.nmvjointcost);
#endif

  memcpy(cc->nmvcosts[0], cpi->nmvcosts[0],
         MV_VALS * sizeof(*cpi->nmvcosts[0]));
  memcpy(cc->nmvcosts[1], cpi->nmvcosts[1],
         MV_VALS * sizeof(*cpi->nmvcosts[1]));
  memcpy(cc->nmvcosts_hp[0], cpi->nmvcosts_hp[0],
         MV_VALS * sizeof(*cpi->nmvcosts_hp[0]));
  memcpy(cc->nmvcosts_hp[1], cpi->nmvcosts_hp[1],
         MV_VALS * sizeof(*cpi->nmvcosts_hp[1]));

#if !CONFIG_MISC_FIXES
  av1_copy(cc->segment_pred_probs, cm->segp.pred_probs);
#endif

  memcpy(cpi->coding_context.last_frame_seg_map_copy, cm->last_frame_seg_map,
         (cm->mi_rows * cm->mi_cols));

  av1_copy(cc->last_ref_lf_deltas, cm->lf.last_ref_deltas);
  av1_copy(cc->last_mode_lf_deltas, cm->lf.last_mode_deltas);

  cc->fc = *cm->fc;
}

static void restore_coding_context(AV1_COMP *cpi) {
  CODING_CONTEXT *const cc = &cpi->coding_context;
  AV1_COMMON *cm = &cpi->common;
#if CONFIG_REF_MV
  int i;
#endif

// Restore key state variables to the snapshot state stored in the
// previous call to av1_save_coding_context.
#if CONFIG_REF_MV
  for (i = 0; i < NMV_CONTEXTS; ++i) {
    av1_copy(cpi->td.mb.nmv_vec_cost[i], cc->nmv_vec_cost[i]);
    memcpy(cpi->nmv_costs[i][0], cc->nmv_costs[i][0],
           MV_VALS * sizeof(*cc->nmv_costs[i][0]));
    memcpy(cpi->nmv_costs[i][1], cc->nmv_costs[i][1],
           MV_VALS * sizeof(*cc->nmv_costs[i][1]));
    memcpy(cpi->nmv_costs_hp[i][0], cc->nmv_costs_hp[i][0],
           MV_VALS * sizeof(*cc->nmv_costs_hp[i][0]));
    memcpy(cpi->nmv_costs_hp[i][1], cc->nmv_costs_hp[i][1],
           MV_VALS * sizeof(*cc->nmv_costs_hp[i][1]));
  }
#else
  av1_copy(cpi->td.mb.nmvjointcost, cc->nmvjointcost);
#endif

  memcpy(cpi->nmvcosts[0], cc->nmvcosts[0], MV_VALS * sizeof(*cc->nmvcosts[0]));
  memcpy(cpi->nmvcosts[1], cc->nmvcosts[1], MV_VALS * sizeof(*cc->nmvcosts[1]));
  memcpy(cpi->nmvcosts_hp[0], cc->nmvcosts_hp[0],
         MV_VALS * sizeof(*cc->nmvcosts_hp[0]));
  memcpy(cpi->nmvcosts_hp[1], cc->nmvcosts_hp[1],
         MV_VALS * sizeof(*cc->nmvcosts_hp[1]));

#if !CONFIG_MISC_FIXES
  av1_copy(cm->segp.pred_probs, cc->segment_pred_probs);
#endif

  memcpy(cm->last_frame_seg_map, cpi->coding_context.last_frame_seg_map_copy,
         (cm->mi_rows * cm->mi_cols));

  av1_copy(cm->lf.last_ref_deltas, cc->last_ref_lf_deltas);
  av1_copy(cm->lf.last_mode_deltas, cc->last_mode_lf_deltas);

  *cm->fc = cc->fc;
}

static void configure_static_seg_features(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  const RATE_CONTROL *const rc = &cpi->rc;
  struct segmentation *const seg = &cm->seg;

  int high_q = (int)(rc->avg_q > 48.0);
  int qi_delta;

  // Disable and clear down for KF
  if (cm->frame_type == KEY_FRAME) {
    // Clear down the global segmentation map
    memset(cpi->segmentation_map, 0, cm->mi_rows * cm->mi_cols);
    seg->update_map = 0;
    seg->update_data = 0;
    cpi->static_mb_pct = 0;

    // Disable segmentation
    av1_disable_segmentation(seg);

    // Clear down the segment features.
    av1_clearall_segfeatures(seg);
  } else if (cpi->refresh_alt_ref_frame) {
    // If this is an alt ref frame
    // Clear down the global segmentation map
    memset(cpi->segmentation_map, 0, cm->mi_rows * cm->mi_cols);
    seg->update_map = 0;
    seg->update_data = 0;
    cpi->static_mb_pct = 0;

    // Disable segmentation and individual segment features by default
    av1_disable_segmentation(seg);
    av1_clearall_segfeatures(seg);

    // Scan frames from current to arf frame.
    // This function re-enables segmentation if appropriate.
    av1_update_mbgraph_stats(cpi);

    // If segmentation was enabled set those features needed for the
    // arf itself.
    if (seg->enabled) {
      seg->update_map = 1;
      seg->update_data = 1;

      qi_delta =
          av1_compute_qdelta(rc, rc->avg_q, rc->avg_q * 0.875, cm->bit_depth);
      av1_set_segdata(seg, 1, SEG_LVL_ALT_Q, qi_delta - 2);
      av1_set_segdata(seg, 1, SEG_LVL_ALT_LF, -2);

      av1_enable_segfeature(seg, 1, SEG_LVL_ALT_Q);
      av1_enable_segfeature(seg, 1, SEG_LVL_ALT_LF);

      // Where relevant assume segment data is delta data
      seg->abs_delta = SEGMENT_DELTADATA;
    }
  } else if (seg->enabled) {
    // All other frames if segmentation has been enabled

    // First normal frame in a valid gf or alt ref group
    if (rc->frames_since_golden == 0) {
      // Set up segment features for normal frames in an arf group
      if (rc->source_alt_ref_active) {
        seg->update_map = 0;
        seg->update_data = 1;
        seg->abs_delta = SEGMENT_DELTADATA;

        qi_delta =
            av1_compute_qdelta(rc, rc->avg_q, rc->avg_q * 1.125, cm->bit_depth);
        av1_set_segdata(seg, 1, SEG_LVL_ALT_Q, qi_delta + 2);
        av1_enable_segfeature(seg, 1, SEG_LVL_ALT_Q);

        av1_set_segdata(seg, 1, SEG_LVL_ALT_LF, -2);
        av1_enable_segfeature(seg, 1, SEG_LVL_ALT_LF);

        // Segment coding disabled for compred testing
        if (high_q || (cpi->static_mb_pct == 100)) {
          av1_set_segdata(seg, 1, SEG_LVL_REF_FRAME, ALTREF_FRAME);
          av1_enable_segfeature(seg, 1, SEG_LVL_REF_FRAME);
          av1_enable_segfeature(seg, 1, SEG_LVL_SKIP);
        }
      } else {
        // Disable segmentation and clear down features if alt ref
        // is not active for this group

        av1_disable_segmentation(seg);

        memset(cpi->segmentation_map, 0, cm->mi_rows * cm->mi_cols);

        seg->update_map = 0;
        seg->update_data = 0;

        av1_clearall_segfeatures(seg);
      }
    } else if (rc->is_src_frame_alt_ref) {
      // Special case where we are coding over the top of a previous
      // alt ref frame.
      // Segment coding disabled for compred testing

      // Enable ref frame features for segment 0 as well
      av1_enable_segfeature(seg, 0, SEG_LVL_REF_FRAME);
      av1_enable_segfeature(seg, 1, SEG_LVL_REF_FRAME);

      // All mbs should use ALTREF_FRAME
      av1_clear_segdata(seg, 0, SEG_LVL_REF_FRAME);
      av1_set_segdata(seg, 0, SEG_LVL_REF_FRAME, ALTREF_FRAME);
      av1_clear_segdata(seg, 1, SEG_LVL_REF_FRAME);
      av1_set_segdata(seg, 1, SEG_LVL_REF_FRAME, ALTREF_FRAME);

      // Skip all MBs if high Q (0,0 mv and skip coeffs)
      if (high_q) {
        av1_enable_segfeature(seg, 0, SEG_LVL_SKIP);
        av1_enable_segfeature(seg, 1, SEG_LVL_SKIP);
      }
      // Enable data update
      seg->update_data = 1;
    } else {
      // All other frames.

      // No updates.. leave things as they are.
      seg->update_map = 0;
      seg->update_data = 0;
    }
  }
}

static void update_reference_segmentation_map(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  MODE_INFO **mi_8x8_ptr = cm->mi_grid_visible;
  uint8_t *cache_ptr = cm->last_frame_seg_map;
  int row, col;

  for (row = 0; row < cm->mi_rows; row++) {
    MODE_INFO **mi_8x8 = mi_8x8_ptr;
    uint8_t *cache = cache_ptr;
    for (col = 0; col < cm->mi_cols; col++, mi_8x8++, cache++)
      cache[0] = mi_8x8[0]->mbmi.segment_id;
    mi_8x8_ptr += cm->mi_stride;
    cache_ptr += cm->mi_cols;
  }
}

static void alloc_raw_frame_buffers(AV1_COMP *cpi) {
  AV1_COMMON *cm = &cpi->common;
  const AV1EncoderConfig *oxcf = &cpi->oxcf;

  if (!cpi->lookahead)
    cpi->lookahead = av1_lookahead_init(oxcf->width, oxcf->height,
                                        cm->subsampling_x, cm->subsampling_y,
#if CONFIG_AOM_HIGHBITDEPTH
                                        cm->use_highbitdepth,
#endif
                                        oxcf->lag_in_frames);
  if (!cpi->lookahead)
    aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                       "Failed to allocate lag buffers");

  // TODO(agrange) Check if ARF is enabled and skip allocation if not.
  if (aom_realloc_frame_buffer(&cpi->alt_ref_buffer, oxcf->width, oxcf->height,
                               cm->subsampling_x, cm->subsampling_y,
#if CONFIG_AOM_HIGHBITDEPTH
                               cm->use_highbitdepth,
#endif
                               AOM_BORDER_IN_PIXELS, cm->byte_alignment, NULL,
                               NULL, NULL))
    aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                       "Failed to allocate altref buffer");
}

static void alloc_util_frame_buffers(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  if (aom_realloc_frame_buffer(&cpi->last_frame_uf, cm->width, cm->height,
                               cm->subsampling_x, cm->subsampling_y,
#if CONFIG_AOM_HIGHBITDEPTH
                               cm->use_highbitdepth,
#endif
                               AOM_BORDER_IN_PIXELS, cm->byte_alignment, NULL,
                               NULL, NULL))
    aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                       "Failed to allocate last frame buffer");

  if (aom_realloc_frame_buffer(&cpi->scaled_source, cm->width, cm->height,
                               cm->subsampling_x, cm->subsampling_y,
#if CONFIG_AOM_HIGHBITDEPTH
                               cm->use_highbitdepth,
#endif
                               AOM_BORDER_IN_PIXELS, cm->byte_alignment, NULL,
                               NULL, NULL))
    aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                       "Failed to allocate scaled source buffer");

  if (aom_realloc_frame_buffer(&cpi->scaled_last_source, cm->width, cm->height,
                               cm->subsampling_x, cm->subsampling_y,
#if CONFIG_AOM_HIGHBITDEPTH
                               cm->use_highbitdepth,
#endif
                               AOM_BORDER_IN_PIXELS, cm->byte_alignment, NULL,
                               NULL, NULL))
    aom_internal_error(&cm->error, AOM_CODEC_MEM_ERROR,
                       "Failed to allocate scaled last source buffer");
}

static int alloc_context_buffers_ext(AV1_COMP *cpi) {
  AV1_COMMON *cm = &cpi->common;
  int mi_size = cm->mi_cols * cm->mi_rows;

  cpi->mbmi_ext_base = aom_calloc(mi_size, sizeof(*cpi->mbmi_ext_base));
  if (!cpi->mbmi_ext_base) return 1;

  return 0;
}

void av1_alloc_compressor_data(AV1_COMP *cpi) {
  AV1_COMMON *cm = &cpi->common;

  av1_alloc_context_buffers(cm, cm->width, cm->height);

  alloc_context_buffers_ext(cpi);

  aom_free(cpi->tile_tok[0][0]);

  {
    unsigned int tokens = get_token_alloc(cm->mb_rows, cm->mb_cols);
    CHECK_MEM_ERROR(cm, cpi->tile_tok[0][0],
                    aom_calloc(tokens, sizeof(*cpi->tile_tok[0][0])));
  }

  av1_setup_pc_tree(&cpi->common, &cpi->td);
}

void av1_new_framerate(AV1_COMP *cpi, double framerate) {
  cpi->framerate = framerate < 0.1 ? 30 : framerate;
  av1_rc_update_framerate(cpi);
}

static void set_tile_limits(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;

  int min_log2_tile_cols, max_log2_tile_cols;
  av1_get_tile_n_bits(cm->mi_cols, &min_log2_tile_cols, &max_log2_tile_cols);

  cm->log2_tile_cols =
      clamp(cpi->oxcf.tile_columns, min_log2_tile_cols, max_log2_tile_cols);
  cm->log2_tile_rows = cpi->oxcf.tile_rows;
}

static void update_frame_size(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  MACROBLOCKD *const xd = &cpi->td.mb.e_mbd;

  av1_set_mb_mi(cm, cm->width, cm->height);
  av1_init_context_buffers(cm);
  av1_init_macroblockd(cm, xd, NULL);
  memset(cpi->mbmi_ext_base, 0,
         cm->mi_rows * cm->mi_cols * sizeof(*cpi->mbmi_ext_base));

  set_tile_limits(cpi);
}

static void init_buffer_indices(AV1_COMP *cpi) {
  cpi->lst_fb_idx = 0;
  cpi->gld_fb_idx = 1;
  cpi->alt_fb_idx = 2;
}

static void init_config(struct AV1_COMP *cpi, AV1EncoderConfig *oxcf) {
  AV1_COMMON *const cm = &cpi->common;

  cpi->oxcf = *oxcf;
  cpi->framerate = oxcf->init_framerate;

  cm->profile = oxcf->profile;
  cm->bit_depth = oxcf->bit_depth;
#if CONFIG_AOM_HIGHBITDEPTH
  cm->use_highbitdepth = oxcf->use_highbitdepth;
#endif
  cm->color_space = oxcf->color_space;
  cm->color_range = oxcf->color_range;

  cm->width = oxcf->width;
  cm->height = oxcf->height;
  av1_alloc_compressor_data(cpi);

  // Single thread case: use counts in common.
  cpi->td.counts = &cm->counts;

  // change includes all joint functionality
  av1_change_config(cpi, oxcf);

  cpi->static_mb_pct = 0;
  cpi->ref_frame_flags = 0;

  init_buffer_indices(cpi);
}

static void set_rc_buffer_sizes(RATE_CONTROL *rc,
                                const AV1EncoderConfig *oxcf) {
  const int64_t bandwidth = oxcf->target_bandwidth;
  const int64_t starting = oxcf->starting_buffer_level_ms;
  const int64_t optimal = oxcf->optimal_buffer_level_ms;
  const int64_t maximum = oxcf->maximum_buffer_size_ms;

  rc->starting_buffer_level = starting * bandwidth / 1000;
  rc->optimal_buffer_level =
      (optimal == 0) ? bandwidth / 8 : optimal * bandwidth / 1000;
  rc->maximum_buffer_size =
      (maximum == 0) ? bandwidth / 8 : maximum * bandwidth / 1000;
}

#if CONFIG_AOM_HIGHBITDEPTH
#define HIGHBD_BFP(BT, SDF, SDAF, VF, SVF, SVAF, SDX3F, SDX8F, SDX4DF) \
  cpi->fn_ptr[BT].sdf = SDF;                                           \
  cpi->fn_ptr[BT].sdaf = SDAF;                                         \
  cpi->fn_ptr[BT].vf = VF;                                             \
  cpi->fn_ptr[BT].svf = SVF;                                           \
  cpi->fn_ptr[BT].svaf = SVAF;                                         \
  cpi->fn_ptr[BT].sdx3f = SDX3F;                                       \
  cpi->fn_ptr[BT].sdx8f = SDX8F;                                       \
  cpi->fn_ptr[BT].sdx4df = SDX4DF;

#define MAKE_BFP_SAD_WRAPPER(fnname)                                           \
  static unsigned int fnname##_bits8(const uint8_t *src_ptr,                   \
                                     int source_stride,                        \
                                     const uint8_t *ref_ptr, int ref_stride) { \
    return fnname(src_ptr, source_stride, ref_ptr, ref_stride);                \
  }                                                                            \
  static unsigned int fnname##_bits10(                                         \
      const uint8_t *src_ptr, int source_stride, const uint8_t *ref_ptr,       \
      int ref_stride) {                                                        \
    return fnname(src_ptr, source_stride, ref_ptr, ref_stride) >> 2;           \
  }                                                                            \
  static unsigned int fnname##_bits12(                                         \
      const uint8_t *src_ptr, int source_stride, const uint8_t *ref_ptr,       \
      int ref_stride) {                                                        \
    return fnname(src_ptr, source_stride, ref_ptr, ref_stride) >> 4;           \
  }

#define MAKE_BFP_SADAVG_WRAPPER(fnname)                                        \
  static unsigned int fnname##_bits8(                                          \
      const uint8_t *src_ptr, int source_stride, const uint8_t *ref_ptr,       \
      int ref_stride, const uint8_t *second_pred) {                            \
    return fnname(src_ptr, source_stride, ref_ptr, ref_stride, second_pred);   \
  }                                                                            \
  static unsigned int fnname##_bits10(                                         \
      const uint8_t *src_ptr, int source_stride, const uint8_t *ref_ptr,       \
      int ref_stride, const uint8_t *second_pred) {                            \
    return fnname(src_ptr, source_stride, ref_ptr, ref_stride, second_pred) >> \
           2;                                                                  \
  }                                                                            \
  static unsigned int fnname##_bits12(                                         \
      const uint8_t *src_ptr, int source_stride, const uint8_t *ref_ptr,       \
      int ref_stride, const uint8_t *second_pred) {                            \
    return fnname(src_ptr, source_stride, ref_ptr, ref_stride, second_pred) >> \
           4;                                                                  \
  }

#define MAKE_BFP_SAD3_WRAPPER(fnname)                                    \
  static void fnname##_bits8(const uint8_t *src_ptr, int source_stride,  \
                             const uint8_t *ref_ptr, int ref_stride,     \
                             unsigned int *sad_array) {                  \
    fnname(src_ptr, source_stride, ref_ptr, ref_stride, sad_array);      \
  }                                                                      \
  static void fnname##_bits10(const uint8_t *src_ptr, int source_stride, \
                              const uint8_t *ref_ptr, int ref_stride,    \
                              unsigned int *sad_array) {                 \
    int i;                                                               \
    fnname(src_ptr, source_stride, ref_ptr, ref_stride, sad_array);      \
    for (i = 0; i < 3; i++) sad_array[i] >>= 2;                          \
  }                                                                      \
  static void fnname##_bits12(const uint8_t *src_ptr, int source_stride, \
                              const uint8_t *ref_ptr, int ref_stride,    \
                              unsigned int *sad_array) {                 \
    int i;                                                               \
    fnname(src_ptr, source_stride, ref_ptr, ref_stride, sad_array);      \
    for (i = 0; i < 3; i++) sad_array[i] >>= 4;                          \
  }

#define MAKE_BFP_SAD8_WRAPPER(fnname)                                    \
  static void fnname##_bits8(const uint8_t *src_ptr, int source_stride,  \
                             const uint8_t *ref_ptr, int ref_stride,     \
                             unsigned int *sad_array) {                  \
    fnname(src_ptr, source_stride, ref_ptr, ref_stride, sad_array);      \
  }                                                                      \
  static void fnname##_bits10(const uint8_t *src_ptr, int source_stride, \
                              const uint8_t *ref_ptr, int ref_stride,    \
                              unsigned int *sad_array) {                 \
    int i;                                                               \
    fnname(src_ptr, source_stride, ref_ptr, ref_stride, sad_array);      \
    for (i = 0; i < 8; i++) sad_array[i] >>= 2;                          \
  }                                                                      \
  static void fnname##_bits12(const uint8_t *src_ptr, int source_stride, \
                              const uint8_t *ref_ptr, int ref_stride,    \
                              unsigned int *sad_array) {                 \
    int i;                                                               \
    fnname(src_ptr, source_stride, ref_ptr, ref_stride, sad_array);      \
    for (i = 0; i < 8; i++) sad_array[i] >>= 4;                          \
  }
#define MAKE_BFP_SAD4D_WRAPPER(fnname)                                        \
  static void fnname##_bits8(const uint8_t *src_ptr, int source_stride,       \
                             const uint8_t *const ref_ptr[], int ref_stride,  \
                             unsigned int *sad_array) {                       \
    fnname(src_ptr, source_stride, ref_ptr, ref_stride, sad_array);           \
  }                                                                           \
  static void fnname##_bits10(const uint8_t *src_ptr, int source_stride,      \
                              const uint8_t *const ref_ptr[], int ref_stride, \
                              unsigned int *sad_array) {                      \
    int i;                                                                    \
    fnname(src_ptr, source_stride, ref_ptr, ref_stride, sad_array);           \
    for (i = 0; i < 4; i++) sad_array[i] >>= 2;                               \
  }                                                                           \
  static void fnname##_bits12(const uint8_t *src_ptr, int source_stride,      \
                              const uint8_t *const ref_ptr[], int ref_stride, \
                              unsigned int *sad_array) {                      \
    int i;                                                                    \
    fnname(src_ptr, source_stride, ref_ptr, ref_stride, sad_array);           \
    for (i = 0; i < 4; i++) sad_array[i] >>= 4;                               \
  }

/* clang-format off */
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad32x16)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad32x16_avg)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad32x16x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad16x32)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad16x32_avg)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad16x32x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad64x32)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad64x32_avg)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad64x32x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad32x64)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad32x64_avg)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad32x64x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad32x32)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad32x32_avg)
MAKE_BFP_SAD3_WRAPPER(aom_highbd_sad32x32x3)
MAKE_BFP_SAD8_WRAPPER(aom_highbd_sad32x32x8)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad32x32x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad64x64)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad64x64_avg)
MAKE_BFP_SAD3_WRAPPER(aom_highbd_sad64x64x3)
MAKE_BFP_SAD8_WRAPPER(aom_highbd_sad64x64x8)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad64x64x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad16x16)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad16x16_avg)
MAKE_BFP_SAD3_WRAPPER(aom_highbd_sad16x16x3)
MAKE_BFP_SAD8_WRAPPER(aom_highbd_sad16x16x8)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad16x16x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad16x8)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad16x8_avg)
MAKE_BFP_SAD3_WRAPPER(aom_highbd_sad16x8x3)
MAKE_BFP_SAD8_WRAPPER(aom_highbd_sad16x8x8)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad16x8x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad8x16)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad8x16_avg)
MAKE_BFP_SAD3_WRAPPER(aom_highbd_sad8x16x3)
MAKE_BFP_SAD8_WRAPPER(aom_highbd_sad8x16x8)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad8x16x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad8x8)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad8x8_avg)
MAKE_BFP_SAD3_WRAPPER(aom_highbd_sad8x8x3)
MAKE_BFP_SAD8_WRAPPER(aom_highbd_sad8x8x8)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad8x8x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad8x4)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad8x4_avg)
MAKE_BFP_SAD8_WRAPPER(aom_highbd_sad8x4x8)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad8x4x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad4x8)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad4x8_avg)
MAKE_BFP_SAD8_WRAPPER(aom_highbd_sad4x8x8)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad4x8x4d)
MAKE_BFP_SAD_WRAPPER(aom_highbd_sad4x4)
MAKE_BFP_SADAVG_WRAPPER(aom_highbd_sad4x4_avg)
MAKE_BFP_SAD3_WRAPPER(aom_highbd_sad4x4x3)
MAKE_BFP_SAD8_WRAPPER(aom_highbd_sad4x4x8)
MAKE_BFP_SAD4D_WRAPPER(aom_highbd_sad4x4x4d)
/* clang-format on */

static void highbd_set_var_fns(AV1_COMP *const cpi) {
  AV1_COMMON *const cm = &cpi->common;
  if (cm->use_highbitdepth) {
    switch (cm->bit_depth) {
      case AOM_BITS_8:
        HIGHBD_BFP(BLOCK_32X16, aom_highbd_sad32x16_bits8,
                   aom_highbd_sad32x16_avg_bits8, aom_highbd_8_variance32x16,
                   aom_highbd_8_sub_pixel_variance32x16,
                   aom_highbd_8_sub_pixel_avg_variance32x16, NULL, NULL,
                   aom_highbd_sad32x16x4d_bits8)

        HIGHBD_BFP(BLOCK_16X32, aom_highbd_sad16x32_bits8,
                   aom_highbd_sad16x32_avg_bits8, aom_highbd_8_variance16x32,
                   aom_highbd_8_sub_pixel_variance16x32,
                   aom_highbd_8_sub_pixel_avg_variance16x32, NULL, NULL,
                   aom_highbd_sad16x32x4d_bits8)

        HIGHBD_BFP(BLOCK_64X32, aom_highbd_sad64x32_bits8,
                   aom_highbd_sad64x32_avg_bits8, aom_highbd_8_variance64x32,
                   aom_highbd_8_sub_pixel_variance64x32,
                   aom_highbd_8_sub_pixel_avg_variance64x32, NULL, NULL,
                   aom_highbd_sad64x32x4d_bits8)

        HIGHBD_BFP(BLOCK_32X64, aom_highbd_sad32x64_bits8,
                   aom_highbd_sad32x64_avg_bits8, aom_highbd_8_variance32x64,
                   aom_highbd_8_sub_pixel_variance32x64,
                   aom_highbd_8_sub_pixel_avg_variance32x64, NULL, NULL,
                   aom_highbd_sad32x64x4d_bits8)

        HIGHBD_BFP(BLOCK_32X32, aom_highbd_sad32x32_bits8,
                   aom_highbd_sad32x32_avg_bits8, aom_highbd_8_variance32x32,
                   aom_highbd_8_sub_pixel_variance32x32,
                   aom_highbd_8_sub_pixel_avg_variance32x32,
                   aom_highbd_sad32x32x3_bits8, aom_highbd_sad32x32x8_bits8,
                   aom_highbd_sad32x32x4d_bits8)

        HIGHBD_BFP(BLOCK_64X64, aom_highbd_sad64x64_bits8,
                   aom_highbd_sad64x64_avg_bits8, aom_highbd_8_variance64x64,
                   aom_highbd_8_sub_pixel_variance64x64,
                   aom_highbd_8_sub_pixel_avg_variance64x64,
                   aom_highbd_sad64x64x3_bits8, aom_highbd_sad64x64x8_bits8,
                   aom_highbd_sad64x64x4d_bits8)

        HIGHBD_BFP(BLOCK_16X16, aom_highbd_sad16x16_bits8,
                   aom_highbd_sad16x16_avg_bits8, aom_highbd_8_variance16x16,
                   aom_highbd_8_sub_pixel_variance16x16,
                   aom_highbd_8_sub_pixel_avg_variance16x16,
                   aom_highbd_sad16x16x3_bits8, aom_highbd_sad16x16x8_bits8,
                   aom_highbd_sad16x16x4d_bits8)

        HIGHBD_BFP(
            BLOCK_16X8, aom_highbd_sad16x8_bits8, aom_highbd_sad16x8_avg_bits8,
            aom_highbd_8_variance16x8, aom_highbd_8_sub_pixel_variance16x8,
            aom_highbd_8_sub_pixel_avg_variance16x8, aom_highbd_sad16x8x3_bits8,
            aom_highbd_sad16x8x8_bits8, aom_highbd_sad16x8x4d_bits8)

        HIGHBD_BFP(
            BLOCK_8X16, aom_highbd_sad8x16_bits8, aom_highbd_sad8x16_avg_bits8,
            aom_highbd_8_variance8x16, aom_highbd_8_sub_pixel_variance8x16,
            aom_highbd_8_sub_pixel_avg_variance8x16, aom_highbd_sad8x16x3_bits8,
            aom_highbd_sad8x16x8_bits8, aom_highbd_sad8x16x4d_bits8)

        HIGHBD_BFP(
            BLOCK_8X8, aom_highbd_sad8x8_bits8, aom_highbd_sad8x8_avg_bits8,
            aom_highbd_8_variance8x8, aom_highbd_8_sub_pixel_variance8x8,
            aom_highbd_8_sub_pixel_avg_variance8x8, aom_highbd_sad8x8x3_bits8,
            aom_highbd_sad8x8x8_bits8, aom_highbd_sad8x8x4d_bits8)

        HIGHBD_BFP(BLOCK_8X4, aom_highbd_sad8x4_bits8,
                   aom_highbd_sad8x4_avg_bits8, aom_highbd_8_variance8x4,
                   aom_highbd_8_sub_pixel_variance8x4,
                   aom_highbd_8_sub_pixel_avg_variance8x4, NULL,
                   aom_highbd_sad8x4x8_bits8, aom_highbd_sad8x4x4d_bits8)

        HIGHBD_BFP(BLOCK_4X8, aom_highbd_sad4x8_bits8,
                   aom_highbd_sad4x8_avg_bits8, aom_highbd_8_variance4x8,
                   aom_highbd_8_sub_pixel_variance4x8,
                   aom_highbd_8_sub_pixel_avg_variance4x8, NULL,
                   aom_highbd_sad4x8x8_bits8, aom_highbd_sad4x8x4d_bits8)

        HIGHBD_BFP(
            BLOCK_4X4, aom_highbd_sad4x4_bits8, aom_highbd_sad4x4_avg_bits8,
            aom_highbd_8_variance4x4, aom_highbd_8_sub_pixel_variance4x4,
            aom_highbd_8_sub_pixel_avg_variance4x4, aom_highbd_sad4x4x3_bits8,
            aom_highbd_sad4x4x8_bits8, aom_highbd_sad4x4x4d_bits8)
        break;

      case AOM_BITS_10:
        HIGHBD_BFP(BLOCK_32X16, aom_highbd_sad32x16_bits10,
                   aom_highbd_sad32x16_avg_bits10, aom_highbd_10_variance32x16,
                   aom_highbd_10_sub_pixel_variance32x16,
                   aom_highbd_10_sub_pixel_avg_variance32x16, NULL, NULL,
                   aom_highbd_sad32x16x4d_bits10)

        HIGHBD_BFP(BLOCK_16X32, aom_highbd_sad16x32_bits10,
                   aom_highbd_sad16x32_avg_bits10, aom_highbd_10_variance16x32,
                   aom_highbd_10_sub_pixel_variance16x32,
                   aom_highbd_10_sub_pixel_avg_variance16x32, NULL, NULL,
                   aom_highbd_sad16x32x4d_bits10)

        HIGHBD_BFP(BLOCK_64X32, aom_highbd_sad64x32_bits10,
                   aom_highbd_sad64x32_avg_bits10, aom_highbd_10_variance64x32,
                   aom_highbd_10_sub_pixel_variance64x32,
                   aom_highbd_10_sub_pixel_avg_variance64x32, NULL, NULL,
                   aom_highbd_sad64x32x4d_bits10)

        HIGHBD_BFP(BLOCK_32X64, aom_highbd_sad32x64_bits10,
                   aom_highbd_sad32x64_avg_bits10, aom_highbd_10_variance32x64,
                   aom_highbd_10_sub_pixel_variance32x64,
                   aom_highbd_10_sub_pixel_avg_variance32x64, NULL, NULL,
                   aom_highbd_sad32x64x4d_bits10)

        HIGHBD_BFP(BLOCK_32X32, aom_highbd_sad32x32_bits10,
                   aom_highbd_sad32x32_avg_bits10, aom_highbd_10_variance32x32,
                   aom_highbd_10_sub_pixel_variance32x32,
                   aom_highbd_10_sub_pixel_avg_variance32x32,
                   aom_highbd_sad32x32x3_bits10, aom_highbd_sad32x32x8_bits10,
                   aom_highbd_sad32x32x4d_bits10)

        HIGHBD_BFP(BLOCK_64X64, aom_highbd_sad64x64_bits10,
                   aom_highbd_sad64x64_avg_bits10, aom_highbd_10_variance64x64,
                   aom_highbd_10_sub_pixel_variance64x64,
                   aom_highbd_10_sub_pixel_avg_variance64x64,
                   aom_highbd_sad64x64x3_bits10, aom_highbd_sad64x64x8_bits10,
                   aom_highbd_sad64x64x4d_bits10)

        HIGHBD_BFP(BLOCK_16X16, aom_highbd_sad16x16_bits10,
                   aom_highbd_sad16x16_avg_bits10, aom_highbd_10_variance16x16,
                   aom_highbd_10_sub_pixel_variance16x16,
                   aom_highbd_10_sub_pixel_avg_variance16x16,
                   aom_highbd_sad16x16x3_bits10, aom_highbd_sad16x16x8_bits10,
                   aom_highbd_sad16x16x4d_bits10)

        HIGHBD_BFP(BLOCK_16X8, aom_highbd_sad16x8_bits10,
                   aom_highbd_sad16x8_avg_bits10, aom_highbd_10_variance16x8,
                   aom_highbd_10_sub_pixel_variance16x8,
                   aom_highbd_10_sub_pixel_avg_variance16x8,
                   aom_highbd_sad16x8x3_bits10, aom_highbd_sad16x8x8_bits10,
                   aom_highbd_sad16x8x4d_bits10)

        HIGHBD_BFP(BLOCK_8X16, aom_highbd_sad8x16_bits10,
                   aom_highbd_sad8x16_avg_bits10, aom_highbd_10_variance8x16,
                   aom_highbd_10_sub_pixel_variance8x16,
                   aom_highbd_10_sub_pixel_avg_variance8x16,
                   aom_highbd_sad8x16x3_bits10, aom_highbd_sad8x16x8_bits10,
                   aom_highbd_sad8x16x4d_bits10)

        HIGHBD_BFP(
            BLOCK_8X8, aom_highbd_sad8x8_bits10, aom_highbd_sad8x8_avg_bits10,
            aom_highbd_10_variance8x8, aom_highbd_10_sub_pixel_variance8x8,
            aom_highbd_10_sub_pixel_avg_variance8x8, aom_highbd_sad8x8x3_bits10,
            aom_highbd_sad8x8x8_bits10, aom_highbd_sad8x8x4d_bits10)

        HIGHBD_BFP(BLOCK_8X4, aom_highbd_sad8x4_bits10,
                   aom_highbd_sad8x4_avg_bits10, aom_highbd_10_variance8x4,
                   aom_highbd_10_sub_pixel_variance8x4,
                   aom_highbd_10_sub_pixel_avg_variance8x4, NULL,
                   aom_highbd_sad8x4x8_bits10, aom_highbd_sad8x4x4d_bits10)

        HIGHBD_BFP(BLOCK_4X8, aom_highbd_sad4x8_bits10,
                   aom_highbd_sad4x8_avg_bits10, aom_highbd_10_variance4x8,
                   aom_highbd_10_sub_pixel_variance4x8,
                   aom_highbd_10_sub_pixel_avg_variance4x8, NULL,
                   aom_highbd_sad4x8x8_bits10, aom_highbd_sad4x8x4d_bits10)

        HIGHBD_BFP(
            BLOCK_4X4, aom_highbd_sad4x4_bits10, aom_highbd_sad4x4_avg_bits10,
            aom_highbd_10_variance4x4, aom_highbd_10_sub_pixel_variance4x4,
            aom_highbd_10_sub_pixel_avg_variance4x4, aom_highbd_sad4x4x3_bits10,
            aom_highbd_sad4x4x8_bits10, aom_highbd_sad4x4x4d_bits10)
        break;

      case AOM_BITS_12:
        HIGHBD_BFP(BLOCK_32X16, aom_highbd_sad32x16_bits12,
                   aom_highbd_sad32x16_avg_bits12, aom_highbd_12_variance32x16,
                   aom_highbd_12_sub_pixel_variance32x16,
                   aom_highbd_12_sub_pixel_avg_variance32x16, NULL, NULL,
                   aom_highbd_sad32x16x4d_bits12)

        HIGHBD_BFP(BLOCK_16X32, aom_highbd_sad16x32_bits12,
                   aom_highbd_sad16x32_avg_bits12, aom_highbd_12_variance16x32,
                   aom_highbd_12_sub_pixel_variance16x32,
                   aom_highbd_12_sub_pixel_avg_variance16x32, NULL, NULL,
                   aom_highbd_sad16x32x4d_bits12)

        HIGHBD_BFP(BLOCK_64X32, aom_highbd_sad64x32_bits12,
                   aom_highbd_sad64x32_avg_bits12, aom_highbd_12_variance64x32,
                   aom_highbd_12_sub_pixel_variance64x32,
                   aom_highbd_12_sub_pixel_avg_variance64x32, NULL, NULL,
                   aom_highbd_sad64x32x4d_bits12)

        HIGHBD_BFP(BLOCK_32X64, aom_highbd_sad32x64_bits12,
                   aom_highbd_sad32x64_avg_bits12, aom_highbd_12_variance32x64,
                   aom_highbd_12_sub_pixel_variance32x64,
                   aom_highbd_12_sub_pixel_avg_variance32x64, NULL, NULL,
                   aom_highbd_sad32x64x4d_bits12)

        HIGHBD_BFP(BLOCK_32X32, aom_highbd_sad32x32_bits12,
                   aom_highbd_sad32x32_avg_bits12, aom_highbd_12_variance32x32,
                   aom_highbd_12_sub_pixel_variance32x32,
                   aom_highbd_12_sub_pixel_avg_variance32x32,
                   aom_highbd_sad32x32x3_bits12, aom_highbd_sad32x32x8_bits12,
                   aom_highbd_sad32x32x4d_bits12)

        HIGHBD_BFP(BLOCK_64X64, aom_highbd_sad64x64_bits12,
                   aom_highbd_sad64x64_avg_bits12, aom_highbd_12_variance64x64,
                   aom_highbd_12_sub_pixel_variance64x64,
                   aom_highbd_12_sub_pixel_avg_variance64x64,
                   aom_highbd_sad64x64x3_bits12, aom_highbd_sad64x64x8_bits12,
                   aom_highbd_sad64x64x4d_bits12)

        HIGHBD_BFP(BLOCK_16X16, aom_highbd_sad16x16_bits12,
                   aom_highbd_sad16x16_avg_bits12, aom_highbd_12_variance16x16,
                   aom_highbd_12_sub_pixel_variance16x16,
                   aom_highbd_12_sub_pixel_avg_variance16x16,
                   aom_highbd_sad16x16x3_bits12, aom_highbd_sad16x16x8_bits12,
                   aom_highbd_sad16x16x4d_bits12)

        HIGHBD_BFP(BLOCK_16X8, aom_highbd_sad16x8_bits12,
                   aom_highbd_sad16x8_avg_bits12, aom_highbd_12_variance16x8,
                   aom_highbd_12_sub_pixel_variance16x8,
                   aom_highbd_12_sub_pixel_avg_variance16x8,
                   aom_highbd_sad16x8x3_bits12, aom_highbd_sad16x8x8_bits12,
                   aom_highbd_sad16x8x4d_bits12)

        HIGHBD_BFP(BLOCK_8X16, aom_highbd_sad8x16_bits12,
                   aom_highbd_sad8x16_avg_bits12, aom_highbd_12_variance8x16,
                   aom_highbd_12_sub_pixel_variance8x16,
                   aom_highbd_12_sub_pixel_avg_variance8x16,
                   aom_highbd_sad8x16x3_bits12, aom_highbd_sad8x16x8_bits12,
                   aom_highbd_sad8x16x4d_bits12)

        HIGHBD_BFP(
            BLOCK_8X8, aom_highbd_sad8x8_bits12, aom_highbd_sad8x8_avg_bits12,
            aom_highbd_12_variance8x8, aom_highbd_12_sub_pixel_variance8x8,
            aom_highbd_12_sub_pixel_avg_variance8x8, aom_highbd_sad8x8x3_bits12,
            aom_highbd_sad8x8x8_bits12, aom_highbd_sad8x8x4d_bits12)

        HIGHBD_BFP(BLOCK_8X4, aom_highbd_sad8x4_bits12,
                   aom_highbd_sad8x4_avg_bits12, aom_highbd_12_variance8x4,
                   aom_highbd_12_sub_pixel_variance8x4,
                   aom_highbd_12_sub_pixel_avg_variance8x4, NULL,
                   aom_highbd_sad8x4x8_bits12, aom_highbd_sad8x4x4d_bits12)

        HIGHBD_BFP(BLOCK_4X8, aom_highbd_sad4x8_bits12,
                   aom_highbd_sad4x8_avg_bits12, aom_highbd_12_variance4x8,
                   aom_highbd_12_sub_pixel_variance4x8,
                   aom_highbd_12_sub_pixel_avg_variance4x8, NULL,
                   aom_highbd_sad4x8x8_bits12, aom_highbd_sad4x8x4d_bits12)

        HIGHBD_BFP(
            BLOCK_4X4, aom_highbd_sad4x4_bits12, aom_highbd_sad4x4_avg_bits12,
            aom_highbd_12_variance4x4, aom_highbd_12_sub_pixel_variance4x4,
            aom_highbd_12_sub_pixel_avg_variance4x4, aom_highbd_sad4x4x3_bits12,
            aom_highbd_sad4x4x8_bits12, aom_highbd_sad4x4x4d_bits12)
        break;

      default:
        assert(0 &&
               "cm->bit_depth should be AOM_BITS_8, "
               "AOM_BITS_10 or AOM_BITS_12");
    }
  }
}
#endif  // CONFIG_AOM_HIGHBITDEPTH

static void realloc_segmentation_maps(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;

  // Create the encoder segmentation map and set all entries to 0
  aom_free(cpi->segmentation_map);
  CHECK_MEM_ERROR(cm, cpi->segmentation_map,
                  aom_calloc(cm->mi_rows * cm->mi_cols, 1));

  // Create a map used for cyclic background refresh.
  if (cpi->cyclic_refresh) av1_cyclic_refresh_free(cpi->cyclic_refresh);
  CHECK_MEM_ERROR(cm, cpi->cyclic_refresh,
                  av1_cyclic_refresh_alloc(cm->mi_rows, cm->mi_cols));

  // Create a map used to mark inactive areas.
  aom_free(cpi->active_map.map);
  CHECK_MEM_ERROR(cm, cpi->active_map.map,
                  aom_calloc(cm->mi_rows * cm->mi_cols, 1));

  // And a place holder structure is the coding context
  // for use if we want to save and restore it
  aom_free(cpi->coding_context.last_frame_seg_map_copy);
  CHECK_MEM_ERROR(cm, cpi->coding_context.last_frame_seg_map_copy,
                  aom_calloc(cm->mi_rows * cm->mi_cols, 1));
}

void av1_change_config(struct AV1_COMP *cpi, const AV1EncoderConfig *oxcf) {
  AV1_COMMON *const cm = &cpi->common;
  RATE_CONTROL *const rc = &cpi->rc;

  if (cm->profile != oxcf->profile) cm->profile = oxcf->profile;
  cm->bit_depth = oxcf->bit_depth;
  cm->color_space = oxcf->color_space;
  cm->color_range = oxcf->color_range;

  if (cm->profile <= PROFILE_1)
    assert(cm->bit_depth == AOM_BITS_8);
  else
    assert(cm->bit_depth > AOM_BITS_8);

  cpi->oxcf = *oxcf;
#if CONFIG_AOM_HIGHBITDEPTH
  cpi->td.mb.e_mbd.bd = (int)cm->bit_depth;
#endif  // CONFIG_AOM_HIGHBITDEPTH

  if ((oxcf->pass == 0) && (oxcf->rc_mode == AOM_Q)) {
    rc->baseline_gf_interval = FIXED_GF_INTERVAL;
  } else {
    rc->baseline_gf_interval = (MIN_GF_INTERVAL + MAX_GF_INTERVAL) / 2;
  }

  cpi->refresh_golden_frame = 0;
  cpi->refresh_last_frame = 1;
  cm->refresh_frame_context = oxcf->error_resilient_mode
                                  ? REFRESH_FRAME_CONTEXT_OFF
                                  : oxcf->frame_parallel_decoding_mode
                                        ? REFRESH_FRAME_CONTEXT_FORWARD
                                        : REFRESH_FRAME_CONTEXT_BACKWARD;
  cm->reset_frame_context = RESET_FRAME_CONTEXT_NONE;

  av1_reset_segment_features(cm);
  av1_set_high_precision_mv(cpi, 0);

  {
    int i;

    for (i = 0; i < MAX_SEGMENTS; i++)
      cpi->segment_encode_breakout[i] = cpi->oxcf.encode_breakout;
  }
  cpi->encode_breakout = cpi->oxcf.encode_breakout;

  set_rc_buffer_sizes(rc, &cpi->oxcf);

  // Under a configuration change, where maximum_buffer_size may change,
  // keep buffer level clipped to the maximum allowed buffer size.
  rc->bits_off_target = AOMMIN(rc->bits_off_target, rc->maximum_buffer_size);
  rc->buffer_level = AOMMIN(rc->buffer_level, rc->maximum_buffer_size);

  // Set up frame rate and related parameters rate control values.
  av1_new_framerate(cpi, cpi->framerate);

  // Set absolute upper and lower quality limits
  rc->worst_quality = cpi->oxcf.worst_allowed_q;
  rc->best_quality = cpi->oxcf.best_allowed_q;

  cm->interp_filter = cpi->sf.default_interp_filter;

  if (cpi->oxcf.render_width > 0 && cpi->oxcf.render_height > 0) {
    cm->render_width = cpi->oxcf.render_width;
    cm->render_height = cpi->oxcf.render_height;
  } else {
    cm->render_width = cpi->oxcf.width;
    cm->render_height = cpi->oxcf.height;
  }
  cm->width = cpi->oxcf.width;
  cm->height = cpi->oxcf.height;

  if (cpi->initial_width) {
    if (cm->width > cpi->initial_width || cm->height > cpi->initial_height) {
      av1_free_context_buffers(cm);
      av1_alloc_compressor_data(cpi);
      realloc_segmentation_maps(cpi);
      cpi->initial_width = cpi->initial_height = 0;
    }
  }
  update_frame_size(cpi);

  cpi->alt_ref_source = NULL;
  rc->is_src_frame_alt_ref = 0;

#if 0
  // Experimental RD Code
  cpi->frame_distortion = 0;
  cpi->last_frame_distortion = 0;
#endif

  set_tile_limits(cpi);

  cpi->ext_refresh_frame_flags_pending = 0;
  cpi->ext_refresh_frame_context_pending = 0;

#if CONFIG_AOM_HIGHBITDEPTH
  highbd_set_var_fns(cpi);
#endif
}

#ifndef M_LOG2_E
#define M_LOG2_E 0.693147180559945309417
#endif
#define log2f(x) (log(x) / (float)M_LOG2_E)

#if !CONFIG_REF_MV
static void cal_nmvjointsadcost(int *mvjointsadcost) {
  mvjointsadcost[0] = 600;
  mvjointsadcost[1] = 300;
  mvjointsadcost[2] = 300;
  mvjointsadcost[3] = 300;
}
#endif

static void cal_nmvsadcosts(int *mvsadcost[2]) {
  int i = 1;

  mvsadcost[0][0] = 0;
  mvsadcost[1][0] = 0;

  do {
    double z = 256 * (2 * (log2f(8 * i) + .6));
    mvsadcost[0][i] = (int)z;
    mvsadcost[1][i] = (int)z;
    mvsadcost[0][-i] = (int)z;
    mvsadcost[1][-i] = (int)z;
  } while (++i <= MV_MAX);
}

static void cal_nmvsadcosts_hp(int *mvsadcost[2]) {
  int i = 1;

  mvsadcost[0][0] = 0;
  mvsadcost[1][0] = 0;

  do {
    double z = 256 * (2 * (log2f(8 * i) + .6));
    mvsadcost[0][i] = (int)z;
    mvsadcost[1][i] = (int)z;
    mvsadcost[0][-i] = (int)z;
    mvsadcost[1][-i] = (int)z;
  } while (++i <= MV_MAX);
}

AV1_COMP *av1_create_compressor(AV1EncoderConfig *oxcf,
                                BufferPool *const pool) {
  unsigned int i;
  AV1_COMP *volatile const cpi = aom_memalign(32, sizeof(AV1_COMP));
  AV1_COMMON *volatile const cm = cpi != NULL ? &cpi->common : NULL;

  if (!cm) return NULL;

  av1_zero(*cpi);

  if (setjmp(cm->error.jmp)) {
    cm->error.setjmp = 0;
    av1_remove_compressor(cpi);
    return 0;
  }

  cm->error.setjmp = 1;
  cm->alloc_mi = av1_enc_alloc_mi;
  cm->free_mi = av1_enc_free_mi;
  cm->setup_mi = av1_enc_setup_mi;

  CHECK_MEM_ERROR(cm, cm->fc, (FRAME_CONTEXT *)aom_calloc(1, sizeof(*cm->fc)));
  CHECK_MEM_ERROR(
      cm, cm->frame_contexts,
      (FRAME_CONTEXT *)aom_calloc(FRAME_CONTEXTS, sizeof(*cm->frame_contexts)));

  cpi->resize_state = 0;
  cpi->resize_avg_qp = 0;
  cpi->resize_buffer_underflow = 0;
  cpi->common.buffer_pool = pool;

  init_config(cpi, oxcf);
  av1_rc_init(&cpi->oxcf, oxcf->pass, &cpi->rc);

  cm->current_video_frame = 0;
  cpi->partition_search_skippable_frame = 0;
  cpi->tile_data = NULL;

  realloc_segmentation_maps(cpi);
#if CONFIG_REF_MV
  for (i = 0; i < NMV_CONTEXTS; ++i) {
    CHECK_MEM_ERROR(cm, cpi->nmv_costs[i][0],
                    aom_calloc(MV_VALS, sizeof(*cpi->nmv_costs[i][0])));
    CHECK_MEM_ERROR(cm, cpi->nmv_costs[i][1],
                    aom_calloc(MV_VALS, sizeof(*cpi->nmv_costs[i][1])));
    CHECK_MEM_ERROR(cm, cpi->nmv_costs_hp[i][0],
                    aom_calloc(MV_VALS, sizeof(*cpi->nmv_costs_hp[i][0])));
    CHECK_MEM_ERROR(cm, cpi->nmv_costs_hp[i][1],
                    aom_calloc(MV_VALS, sizeof(*cpi->nmv_costs_hp[i][1])));
  }
#endif

  CHECK_MEM_ERROR(cm, cpi->nmvcosts[0],
                  aom_calloc(MV_VALS, sizeof(*cpi->nmvcosts[0])));
  CHECK_MEM_ERROR(cm, cpi->nmvcosts[1],
                  aom_calloc(MV_VALS, sizeof(*cpi->nmvcosts[1])));
  CHECK_MEM_ERROR(cm, cpi->nmvcosts_hp[0],
                  aom_calloc(MV_VALS, sizeof(*cpi->nmvcosts_hp[0])));
  CHECK_MEM_ERROR(cm, cpi->nmvcosts_hp[1],
                  aom_calloc(MV_VALS, sizeof(*cpi->nmvcosts_hp[1])));
  CHECK_MEM_ERROR(cm, cpi->nmvsadcosts[0],
                  aom_calloc(MV_VALS, sizeof(*cpi->nmvsadcosts[0])));
  CHECK_MEM_ERROR(cm, cpi->nmvsadcosts[1],
                  aom_calloc(MV_VALS, sizeof(*cpi->nmvsadcosts[1])));
  CHECK_MEM_ERROR(cm, cpi->nmvsadcosts_hp[0],
                  aom_calloc(MV_VALS, sizeof(*cpi->nmvsadcosts_hp[0])));
  CHECK_MEM_ERROR(cm, cpi->nmvsadcosts_hp[1],
                  aom_calloc(MV_VALS, sizeof(*cpi->nmvsadcosts_hp[1])));

  for (i = 0; i < (sizeof(cpi->mbgraph_stats) / sizeof(cpi->mbgraph_stats[0]));
       i++) {
    CHECK_MEM_ERROR(
        cm, cpi->mbgraph_stats[i].mb_stats,
        aom_calloc(cm->MBs * sizeof(*cpi->mbgraph_stats[i].mb_stats), 1));
  }

#if CONFIG_FP_MB_STATS
  cpi->use_fp_mb_stats = 0;
  if (cpi->use_fp_mb_stats) {
    // a place holder used to store the first pass mb stats in the first pass
    CHECK_MEM_ERROR(cm, cpi->twopass.frame_mb_stats_buf,
                    aom_calloc(cm->MBs * sizeof(uint8_t), 1));
  } else {
    cpi->twopass.frame_mb_stats_buf = NULL;
  }
#endif

  cpi->refresh_alt_ref_frame = 0;
  cpi->multi_arf_last_grp_enabled = 0;

  cpi->b_calculate_psnr = CONFIG_INTERNAL_STATS;
#if CONFIG_INTERNAL_STATS
  cpi->b_calculate_blockiness = 1;
  cpi->b_calculate_consistency = 1;
  cpi->total_inconsistency = 0;
  cpi->psnr.worst = 100.0;
  cpi->worst_ssim = 100.0;

  cpi->count = 0;
  cpi->bytes = 0;

  if (cpi->b_calculate_psnr) {
    cpi->total_sq_error = 0;
    cpi->total_samples = 0;

    cpi->tot_recode_hits = 0;
    cpi->summed_quality = 0;
    cpi->summed_weights = 0;
  }

  cpi->fastssim.worst = 100.0;
  cpi->psnrhvs.worst = 100.0;

  if (cpi->b_calculate_blockiness) {
    cpi->total_blockiness = 0;
    cpi->worst_blockiness = 0.0;
  }

  if (cpi->b_calculate_consistency) {
    cpi->ssim_vars = aom_malloc(sizeof(*cpi->ssim_vars) * 4 *
                                cpi->common.mi_rows * cpi->common.mi_cols);
    cpi->worst_consistency = 100.0;
  }
#endif

  cpi->first_time_stamp_ever = INT64_MAX;

#if CONFIG_REF_MV
  for (i = 0; i < NMV_CONTEXTS; ++i) {
    cpi->td.mb.nmvcost[i][0] = &cpi->nmv_costs[i][0][MV_MAX];
    cpi->td.mb.nmvcost[i][1] = &cpi->nmv_costs[i][1][MV_MAX];
    cpi->td.mb.nmvcost_hp[i][0] = &cpi->nmv_costs_hp[i][0][MV_MAX];
    cpi->td.mb.nmvcost_hp[i][1] = &cpi->nmv_costs_hp[i][1][MV_MAX];
  }
#else
  cal_nmvjointsadcost(cpi->td.mb.nmvjointsadcost);
  cpi->td.mb.nmvcost[0] = &cpi->nmvcosts[0][MV_MAX];
  cpi->td.mb.nmvcost[1] = &cpi->nmvcosts[1][MV_MAX];
  cpi->td.mb.nmvcost_hp[0] = &cpi->nmvcosts_hp[0][MV_MAX];
  cpi->td.mb.nmvcost_hp[1] = &cpi->nmvcosts_hp[1][MV_MAX];
#endif

  cpi->td.mb.nmvsadcost[0] = &cpi->nmvsadcosts[0][MV_MAX];
  cpi->td.mb.nmvsadcost[1] = &cpi->nmvsadcosts[1][MV_MAX];
  cal_nmvsadcosts(cpi->td.mb.nmvsadcost);

  cpi->td.mb.nmvsadcost_hp[0] = &cpi->nmvsadcosts_hp[0][MV_MAX];
  cpi->td.mb.nmvsadcost_hp[1] = &cpi->nmvsadcosts_hp[1][MV_MAX];
  cal_nmvsadcosts_hp(cpi->td.mb.nmvsadcost_hp);

#ifdef OUTPUT_YUV_SKINMAP
  yuv_skinmap_file = fopen("skinmap.yuv", "ab");
#endif
#ifdef OUTPUT_YUV_REC
  yuv_rec_file = fopen("rec.yuv", "wb");
#endif

#if 0
  framepsnr = fopen("framepsnr.stt", "a");
  kf_list = fopen("kf_list.stt", "w");
#endif

  cpi->allow_encode_breakout = ENCODE_BREAKOUT_ENABLED;

  if (oxcf->pass == 1) {
    av1_init_first_pass(cpi);
  } else if (oxcf->pass == 2) {
    const size_t packet_sz = sizeof(FIRSTPASS_STATS);
    const int packets = (int)(oxcf->two_pass_stats_in.sz / packet_sz);

#if CONFIG_FP_MB_STATS
    if (cpi->use_fp_mb_stats) {
      const size_t psz = cpi->common.MBs * sizeof(uint8_t);
      const int ps = (int)(oxcf->firstpass_mb_stats_in.sz / psz);

      cpi->twopass.firstpass_mb_stats.mb_stats_start =
          oxcf->firstpass_mb_stats_in.buf;
      cpi->twopass.firstpass_mb_stats.mb_stats_end =
          cpi->twopass.firstpass_mb_stats.mb_stats_start +
          (ps - 1) * cpi->common.MBs * sizeof(uint8_t);
    }
#endif

    cpi->twopass.stats_in_start = oxcf->two_pass_stats_in.buf;
    cpi->twopass.stats_in = cpi->twopass.stats_in_start;
    cpi->twopass.stats_in_end = &cpi->twopass.stats_in[packets - 1];

    av1_init_second_pass(cpi);
  }

  av1_set_speed_features_framesize_independent(cpi);
  av1_set_speed_features_framesize_dependent(cpi);

  // Allocate memory to store variances for a frame.
  CHECK_MEM_ERROR(cm, cpi->source_diff_var, aom_calloc(cm->MBs, sizeof(diff)));
  cpi->source_var_thresh = 0;
  cpi->frames_till_next_var_check = 0;

#define BFP(BT, SDF, SDAF, VF, SVF, SVAF, SDX3F, SDX8F, SDX4DF) \
  cpi->fn_ptr[BT].sdf = SDF;                                    \
  cpi->fn_ptr[BT].sdaf = SDAF;                                  \
  cpi->fn_ptr[BT].vf = VF;                                      \
  cpi->fn_ptr[BT].svf = SVF;                                    \
  cpi->fn_ptr[BT].svaf = SVAF;                                  \
  cpi->fn_ptr[BT].sdx3f = SDX3F;                                \
  cpi->fn_ptr[BT].sdx8f = SDX8F;                                \
  cpi->fn_ptr[BT].sdx4df = SDX4DF;

  BFP(BLOCK_32X16, aom_sad32x16, aom_sad32x16_avg, aom_variance32x16,
      aom_sub_pixel_variance32x16, aom_sub_pixel_avg_variance32x16, NULL, NULL,
      aom_sad32x16x4d)

  BFP(BLOCK_16X32, aom_sad16x32, aom_sad16x32_avg, aom_variance16x32,
      aom_sub_pixel_variance16x32, aom_sub_pixel_avg_variance16x32, NULL, NULL,
      aom_sad16x32x4d)

  BFP(BLOCK_64X32, aom_sad64x32, aom_sad64x32_avg, aom_variance64x32,
      aom_sub_pixel_variance64x32, aom_sub_pixel_avg_variance64x32, NULL, NULL,
      aom_sad64x32x4d)

  BFP(BLOCK_32X64, aom_sad32x64, aom_sad32x64_avg, aom_variance32x64,
      aom_sub_pixel_variance32x64, aom_sub_pixel_avg_variance32x64, NULL, NULL,
      aom_sad32x64x4d)

  BFP(BLOCK_32X32, aom_sad32x32, aom_sad32x32_avg, aom_variance32x32,
      aom_sub_pixel_variance32x32, aom_sub_pixel_avg_variance32x32,
      aom_sad32x32x3, aom_sad32x32x8, aom_sad32x32x4d)

  BFP(BLOCK_64X64, aom_sad64x64, aom_sad64x64_avg, aom_variance64x64,
      aom_sub_pixel_variance64x64, aom_sub_pixel_avg_variance64x64,
      aom_sad64x64x3, aom_sad64x64x8, aom_sad64x64x4d)

  BFP(BLOCK_16X16, aom_sad16x16, aom_sad16x16_avg, aom_variance16x16,
      aom_sub_pixel_variance16x16, aom_sub_pixel_avg_variance16x16,
      aom_sad16x16x3, aom_sad16x16x8, aom_sad16x16x4d)

  BFP(BLOCK_16X8, aom_sad16x8, aom_sad16x8_avg, aom_variance16x8,
      aom_sub_pixel_variance16x8, aom_sub_pixel_avg_variance16x8, aom_sad16x8x3,
      aom_sad16x8x8, aom_sad16x8x4d)

  BFP(BLOCK_8X16, aom_sad8x16, aom_sad8x16_avg, aom_variance8x16,
      aom_sub_pixel_variance8x16, aom_sub_pixel_avg_variance8x16, aom_sad8x16x3,
      aom_sad8x16x8, aom_sad8x16x4d)

  BFP(BLOCK_8X8, aom_sad8x8, aom_sad8x8_avg, aom_variance8x8,
      aom_sub_pixel_variance8x8, aom_sub_pixel_avg_variance8x8, aom_sad8x8x3,
      aom_sad8x8x8, aom_sad8x8x4d)

  BFP(BLOCK_8X4, aom_sad8x4, aom_sad8x4_avg, aom_variance8x4,
      aom_sub_pixel_variance8x4, aom_sub_pixel_avg_variance8x4, NULL,
      aom_sad8x4x8, aom_sad8x4x4d)

  BFP(BLOCK_4X8, aom_sad4x8, aom_sad4x8_avg, aom_variance4x8,
      aom_sub_pixel_variance4x8, aom_sub_pixel_avg_variance4x8, NULL,
      aom_sad4x8x8, aom_sad4x8x4d)

  BFP(BLOCK_4X4, aom_sad4x4, aom_sad4x4_avg, aom_variance4x4,
      aom_sub_pixel_variance4x4, aom_sub_pixel_avg_variance4x4, aom_sad4x4x3,
      aom_sad4x4x8, aom_sad4x4x4d)

#if CONFIG_AOM_HIGHBITDEPTH
  highbd_set_var_fns(cpi);
#endif

  /* av1_init_quantizer() is first called here. Add check in
   * av1_frame_init_quantizer() so that av1_init_quantizer is only
   * called later when needed. This will avoid unnecessary calls of
   * av1_init_quantizer() for every frame.
   */
  av1_init_quantizer(cpi);
#if CONFIG_AOM_QM
  aom_qm_init(cm);
#endif

  av1_loop_filter_init(cm);

  cm->error.setjmp = 0;

  return cpi;
}
#define SNPRINT(H, T) snprintf((H) + strlen(H), sizeof(H) - strlen(H), (T))

#define SNPRINT2(H, T, V) \
  snprintf((H) + strlen(H), sizeof(H) - strlen(H), (T), (V))

void av1_remove_compressor(AV1_COMP *cpi) {
  AV1_COMMON *cm;
  unsigned int i;
  int t;

  if (!cpi) return;

  cm = &cpi->common;
  if (cm->current_video_frame > 0) {
#if CONFIG_INTERNAL_STATS
    aom_clear_system_state();

    if (cpi->oxcf.pass != 1) {
      char headings[512] = { 0 };
      char results[512] = { 0 };
      FILE *f = fopen("opsnr.stt", "a");
      double time_encoded =
          (cpi->last_end_time_stamp_seen - cpi->first_time_stamp_ever) /
          10000000.000;
      double total_encode_time =
          (cpi->time_receive_data + cpi->time_compress_data) / 1000.000;
      const double dr =
          (double)cpi->bytes * (double)8 / (double)1000 / time_encoded;
      const double peak = (double)((1 << cpi->oxcf.input_bit_depth) - 1);

      if (cpi->b_calculate_psnr) {
        const double total_psnr = aom_sse_to_psnr(
            (double)cpi->total_samples, peak, (double)cpi->total_sq_error);
        const double total_ssim =
            100 * pow(cpi->summed_quality / cpi->summed_weights, 8.0);

        snprintf(headings, sizeof(headings),
                 "Bitrate\tAVGPsnr\tGLBPsnr\tAVPsnrP\tGLPsnrP\t"
                 "AOMSSIM\tVPSSIMP\tFASTSIM\tPSNRHVS\t"
                 "WstPsnr\tWstSsim\tWstFast\tWstHVS");
        snprintf(results, sizeof(results),
                 "%7.2f\t%7.3f\t%7.3f\t%7.3f\t%7.3f\t"
                 "%7.3f\t%7.3f\t%7.3f\t%7.3f\t"
                 "%7.3f\t%7.3f\t%7.3f\t%7.3f",
                 dr, cpi->psnr.stat[ALL] / cpi->count, total_psnr,
                 cpi->psnr.stat[ALL] / cpi->count, total_psnr, total_ssim,
                 total_ssim, cpi->fastssim.stat[ALL] / cpi->count,
                 cpi->psnrhvs.stat[ALL] / cpi->count, cpi->psnr.worst,
                 cpi->worst_ssim, cpi->fastssim.worst, cpi->psnrhvs.worst);

        if (cpi->b_calculate_blockiness) {
          SNPRINT(headings, "\t  Block\tWstBlck");
          SNPRINT2(results, "\t%7.3f", cpi->total_blockiness / cpi->count);
          SNPRINT2(results, "\t%7.3f", cpi->worst_blockiness);
        }

        if (cpi->b_calculate_consistency) {
          double consistency =
              aom_sse_to_psnr((double)cpi->total_samples, peak,
                              (double)cpi->total_inconsistency);

          SNPRINT(headings, "\tConsist\tWstCons");
          SNPRINT2(results, "\t%7.3f", consistency);
          SNPRINT2(results, "\t%7.3f", cpi->worst_consistency);
        }

        fprintf(f, "%s\t    Time\n", headings);
        fprintf(f, "%s\t%8.0f\n", results, total_encode_time);
      }

      fclose(f);
    }

#endif

#if 0
    {
      printf("\n_pick_loop_filter_level:%d\n", cpi->time_pick_lpf / 1000);
      printf("\n_frames recive_data encod_mb_row compress_frame  Total\n");
      printf("%6d %10ld %10ld %10ld %10ld\n", cpi->common.current_video_frame,
             cpi->time_receive_data / 1000, cpi->time_encode_sb_row / 1000,
             cpi->time_compress_data / 1000,
             (cpi->time_receive_data + cpi->time_compress_data) / 1000);
    }
#endif
  }

  for (t = 0; t < cpi->num_workers; ++t) {
    AVxWorker *const worker = &cpi->workers[t];
    EncWorkerData *const thread_data = &cpi->tile_thr_data[t];

    // Deallocate allocated threads.
    aom_get_worker_interface()->end(worker);

    // Deallocate allocated thread data.
    if (t < cpi->num_workers - 1) {
      aom_free(thread_data->td->counts);
      av1_free_pc_tree(thread_data->td);
      aom_free(thread_data->td);
    }
  }
  aom_free(cpi->tile_thr_data);
  aom_free(cpi->workers);

  if (cpi->num_workers > 1) av1_loop_filter_dealloc(&cpi->lf_row_sync);

  dealloc_compressor_data(cpi);

  for (i = 0; i < sizeof(cpi->mbgraph_stats) / sizeof(cpi->mbgraph_stats[0]);
       ++i) {
    aom_free(cpi->mbgraph_stats[i].mb_stats);
  }

#if CONFIG_FP_MB_STATS
  if (cpi->use_fp_mb_stats) {
    aom_free(cpi->twopass.frame_mb_stats_buf);
    cpi->twopass.frame_mb_stats_buf = NULL;
  }
#endif

  av1_remove_common(cm);
  av1_free_ref_frame_buffers(cm->buffer_pool);
  aom_free(cpi);

#ifdef OUTPUT_YUV_SKINMAP
  fclose(yuv_skinmap_file);
#endif
#ifdef OUTPUT_YUV_REC
  fclose(yuv_rec_file);
#endif

#if 0

  if (keyfile)
    fclose(keyfile);

  if (framepsnr)
    fclose(framepsnr);

  if (kf_list)
    fclose(kf_list);

#endif
}

static void generate_psnr_packet(AV1_COMP *cpi) {
  struct aom_codec_cx_pkt pkt;
  int i;
  PSNR_STATS psnr;
#if CONFIG_AOM_HIGHBITDEPTH
  aom_calc_highbd_psnr(cpi->Source, cpi->common.frame_to_show, &psnr,
                       cpi->td.mb.e_mbd.bd, cpi->oxcf.input_bit_depth);
#else
  aom_calc_psnr(cpi->Source, cpi->common.frame_to_show, &psnr);
#endif

  for (i = 0; i < 4; ++i) {
    pkt.data.psnr.samples[i] = psnr.samples[i];
    pkt.data.psnr.sse[i] = psnr.sse[i];
    pkt.data.psnr.psnr[i] = psnr.psnr[i];
  }
  pkt.kind = AOM_CODEC_PSNR_PKT;
  aom_codec_pkt_list_add(cpi->output_pkt_list, &pkt);
}

int av1_use_as_reference(AV1_COMP *cpi, int ref_frame_flags) {
  if (ref_frame_flags > 7) return -1;

  cpi->ref_frame_flags = ref_frame_flags;
  return 0;
}

void av1_update_reference(AV1_COMP *cpi, int ref_frame_flags) {
  cpi->ext_refresh_golden_frame = (ref_frame_flags & AOM_GOLD_FLAG) != 0;
  cpi->ext_refresh_alt_ref_frame = (ref_frame_flags & AOM_ALT_FLAG) != 0;
  cpi->ext_refresh_last_frame = (ref_frame_flags & AOM_LAST_FLAG) != 0;
  cpi->ext_refresh_frame_flags_pending = 1;
}

static YV12_BUFFER_CONFIG *get_av1_ref_frame_buffer(
    AV1_COMP *cpi, AOM_REFFRAME ref_frame_flag) {
  MV_REFERENCE_FRAME ref_frame = NONE;
  if (ref_frame_flag == AOM_LAST_FLAG)
    ref_frame = LAST_FRAME;
  else if (ref_frame_flag == AOM_GOLD_FLAG)
    ref_frame = GOLDEN_FRAME;
  else if (ref_frame_flag == AOM_ALT_FLAG)
    ref_frame = ALTREF_FRAME;

  return ref_frame == NONE ? NULL : get_ref_frame_buffer(cpi, ref_frame);
}

int av1_copy_reference_enc(AV1_COMP *cpi, AOM_REFFRAME ref_frame_flag,
                           YV12_BUFFER_CONFIG *sd) {
  YV12_BUFFER_CONFIG *cfg = get_av1_ref_frame_buffer(cpi, ref_frame_flag);
  if (cfg) {
    aom_yv12_copy_frame(cfg, sd);
    return 0;
  } else {
    return -1;
  }
}

int av1_set_reference_enc(AV1_COMP *cpi, AOM_REFFRAME ref_frame_flag,
                          YV12_BUFFER_CONFIG *sd) {
  YV12_BUFFER_CONFIG *cfg = get_av1_ref_frame_buffer(cpi, ref_frame_flag);
  if (cfg) {
    aom_yv12_copy_frame(sd, cfg);
    return 0;
  } else {
    return -1;
  }
}

int av1_update_entropy(AV1_COMP *cpi, int update) {
  cpi->ext_refresh_frame_context = update;
  cpi->ext_refresh_frame_context_pending = 1;
  return 0;
}

#if defined(OUTPUT_YUV_DENOISED) || defined(OUTPUT_YUV_SKINMAP)
// The denoiser buffer is allocated as a YUV 440 buffer. This function writes it
// as YUV 420. We simply use the top-left pixels of the UV buffers, since we do
// not denoise the UV channels at this time. If ever we implement UV channel
// denoising we will have to modify this.
void av1_write_yuv_frame_420(YV12_BUFFER_CONFIG *s, FILE *f) {
  uint8_t *src = s->y_buffer;
  int h = s->y_height;

  do {
    fwrite(src, s->y_width, 1, f);
    src += s->y_stride;
  } while (--h);

  src = s->u_buffer;
  h = s->uv_height;

  do {
    fwrite(src, s->uv_width, 1, f);
    src += s->uv_stride;
  } while (--h);

  src = s->v_buffer;
  h = s->uv_height;

  do {
    fwrite(src, s->uv_width, 1, f);
    src += s->uv_stride;
  } while (--h);
}
#endif

#ifdef OUTPUT_YUV_REC
void av1_write_yuv_rec_frame(AV1_COMMON *cm) {
  YV12_BUFFER_CONFIG *s = cm->frame_to_show;
  uint8_t *src = s->y_buffer;
  int h = cm->height;

#if CONFIG_AOM_HIGHBITDEPTH
  if (s->flags & YV12_FLAG_HIGHBITDEPTH) {
    uint16_t *src16 = CONVERT_TO_SHORTPTR(s->y_buffer);

    do {
      fwrite(src16, s->y_width, 2, yuv_rec_file);
      src16 += s->y_stride;
    } while (--h);

    src16 = CONVERT_TO_SHORTPTR(s->u_buffer);
    h = s->uv_height;

    do {
      fwrite(src16, s->uv_width, 2, yuv_rec_file);
      src16 += s->uv_stride;
    } while (--h);

    src16 = CONVERT_TO_SHORTPTR(s->v_buffer);
    h = s->uv_height;

    do {
      fwrite(src16, s->uv_width, 2, yuv_rec_file);
      src16 += s->uv_stride;
    } while (--h);

    fflush(yuv_rec_file);
    return;
  }
#endif  // CONFIG_AOM_HIGHBITDEPTH

  do {
    fwrite(src, s->y_width, 1, yuv_rec_file);
    src += s->y_stride;
  } while (--h);

  src = s->u_buffer;
  h = s->uv_height;

  do {
    fwrite(src, s->uv_width, 1, yuv_rec_file);
    src += s->uv_stride;
  } while (--h);

  src = s->v_buffer;
  h = s->uv_height;

  do {
    fwrite(src, s->uv_width, 1, yuv_rec_file);
    src += s->uv_stride;
  } while (--h);

  fflush(yuv_rec_file);
}
#endif

#if CONFIG_AOM_HIGHBITDEPTH
static void scale_and_extend_frame_nonnormative(const YV12_BUFFER_CONFIG *src,
                                                YV12_BUFFER_CONFIG *dst,
                                                int bd) {
#else
static void scale_and_extend_frame_nonnormative(const YV12_BUFFER_CONFIG *src,
                                                YV12_BUFFER_CONFIG *dst) {
#endif  // CONFIG_AOM_HIGHBITDEPTH
  // TODO(dkovalev): replace YV12_BUFFER_CONFIG with aom_image_t
  int i;
  const uint8_t *const srcs[3] = { src->y_buffer, src->u_buffer,
                                   src->v_buffer };
  const int src_strides[3] = { src->y_stride, src->uv_stride, src->uv_stride };
  const int src_widths[3] = { src->y_crop_width, src->uv_crop_width,
                              src->uv_crop_width };
  const int src_heights[3] = { src->y_crop_height, src->uv_crop_height,
                               src->uv_crop_height };
  uint8_t *const dsts[3] = { dst->y_buffer, dst->u_buffer, dst->v_buffer };
  const int dst_strides[3] = { dst->y_stride, dst->uv_stride, dst->uv_stride };
  const int dst_widths[3] = { dst->y_crop_width, dst->uv_crop_width,
                              dst->uv_crop_width };
  const int dst_heights[3] = { dst->y_crop_height, dst->uv_crop_height,
                               dst->uv_crop_height };

  for (i = 0; i < MAX_MB_PLANE; ++i) {
#if CONFIG_AOM_HIGHBITDEPTH
    if (src->flags & YV12_FLAG_HIGHBITDEPTH) {
      av1_highbd_resize_plane(srcs[i], src_heights[i], src_widths[i],
                              src_strides[i], dsts[i], dst_heights[i],
                              dst_widths[i], dst_strides[i], bd);
    } else {
      av1_resize_plane(srcs[i], src_heights[i], src_widths[i], src_strides[i],
                       dsts[i], dst_heights[i], dst_widths[i], dst_strides[i]);
    }
#else
    av1_resize_plane(srcs[i], src_heights[i], src_widths[i], src_strides[i],
                     dsts[i], dst_heights[i], dst_widths[i], dst_strides[i]);
#endif  // CONFIG_AOM_HIGHBITDEPTH
  }
  aom_extend_frame_borders(dst);
}

#if CONFIG_AOM_HIGHBITDEPTH
static void scale_and_extend_frame(const YV12_BUFFER_CONFIG *src,
                                   YV12_BUFFER_CONFIG *dst, int bd) {
#else
static void scale_and_extend_frame(const YV12_BUFFER_CONFIG *src,
                                   YV12_BUFFER_CONFIG *dst) {
#endif  // CONFIG_AOM_HIGHBITDEPTH
  const int src_w = src->y_crop_width;
  const int src_h = src->y_crop_height;
  const int dst_w = dst->y_crop_width;
  const int dst_h = dst->y_crop_height;
  const uint8_t *const srcs[3] = { src->y_buffer, src->u_buffer,
                                   src->v_buffer };
  const int src_strides[3] = { src->y_stride, src->uv_stride, src->uv_stride };
  uint8_t *const dsts[3] = { dst->y_buffer, dst->u_buffer, dst->v_buffer };
  const int dst_strides[3] = { dst->y_stride, dst->uv_stride, dst->uv_stride };
  InterpFilterParams interp_filter_params = get_interp_filter_params(EIGHTTAP);
  int x, y, i;

  for (y = 0; y < dst_h; y += 16) {
    for (x = 0; x < dst_w; x += 16) {
      for (i = 0; i < MAX_MB_PLANE; ++i) {
        const int factor = (i == 0 || i == 3 ? 1 : 2);
        const int x_q4 = x * (16 / factor) * src_w / dst_w;
        const int y_q4 = y * (16 / factor) * src_h / dst_h;
        const int subpel_x = x_q4 & 0xf;
        const int subpel_y = y_q4 & 0xf;
        const int src_stride = src_strides[i];
        const int dst_stride = dst_strides[i];
        const uint8_t *src_ptr = srcs[i] +
                                 (y / factor) * src_h / dst_h * src_stride +
                                 (x / factor) * src_w / dst_w;
        uint8_t *dst_ptr = dsts[i] + (y / factor) * dst_stride + (x / factor);
        const int16_t *filter_x =
            get_interp_filter_subpel_kernel(interp_filter_params, subpel_x);
        const int16_t *filter_y =
            get_interp_filter_subpel_kernel(interp_filter_params, subpel_y);

#if CONFIG_AOM_HIGHBITDEPTH
        if (src->flags & YV12_FLAG_HIGHBITDEPTH) {
          aom_highbd_convolve8(src_ptr, src_stride, dst_ptr, dst_stride,
                               filter_x, 16 * src_w / dst_w, filter_y,
                               16 * src_h / dst_h, 16 / factor, 16 / factor,
                               bd);
        } else {
          aom_convolve8(src_ptr, src_stride, dst_ptr, dst_stride, filter_x,
                        16 * src_w / dst_w, filter_y, 16 * src_h / dst_h,
                        16 / factor, 16 / factor);
        }
#else
        aom_convolve8(src_ptr, src_stride, dst_ptr, dst_stride, filter_x,
                      16 * src_w / dst_w, filter_y, 16 * src_h / dst_h,
                      16 / factor, 16 / factor);
#endif  // CONFIG_AOM_HIGHBITDEPTH
      }
    }
  }

  aom_extend_frame_borders(dst);
}

static int scale_down(AV1_COMP *cpi, int q) {
  RATE_CONTROL *const rc = &cpi->rc;
  GF_GROUP *const gf_group = &cpi->twopass.gf_group;
  int scale = 0;
  assert(frame_is_kf_gf_arf(cpi));

  if (rc->frame_size_selector == UNSCALED &&
      q >= rc->rf_level_maxq[gf_group->rf_level[gf_group->index]]) {
    const int max_size_thresh =
        (int)(rate_thresh_mult[SCALE_STEP1] *
              AOMMAX(rc->this_frame_target, rc->avg_frame_bandwidth));
    scale = rc->projected_frame_size > max_size_thresh ? 1 : 0;
  }
  return scale;
}

// Function to test for conditions that indicate we should loop
// back and recode a frame.
static int recode_loop_test(AV1_COMP *cpi, int high_limit, int low_limit, int q,
                            int maxq, int minq) {
  const RATE_CONTROL *const rc = &cpi->rc;
  const AV1EncoderConfig *const oxcf = &cpi->oxcf;
  const int frame_is_kfgfarf = frame_is_kf_gf_arf(cpi);
  int force_recode = 0;

  if ((rc->projected_frame_size >= rc->max_frame_bandwidth) ||
      (cpi->sf.recode_loop == ALLOW_RECODE) ||
      (frame_is_kfgfarf && (cpi->sf.recode_loop == ALLOW_RECODE_KFARFGF))) {
    if (frame_is_kfgfarf && (oxcf->resize_mode == RESIZE_DYNAMIC) &&
        scale_down(cpi, q)) {
      // Code this group at a lower resolution.
      cpi->resize_pending = 1;
      return 1;
    }

    // TODO(agrange) high_limit could be greater than the scale-down threshold.
    if ((rc->projected_frame_size > high_limit && q < maxq) ||
        (rc->projected_frame_size < low_limit && q > minq)) {
      force_recode = 1;
    } else if (cpi->oxcf.rc_mode == AOM_CQ) {
      // Deal with frame undershoot and whether or not we are
      // below the automatically set cq level.
      if (q > oxcf->cq_level &&
          rc->projected_frame_size < ((rc->this_frame_target * 7) >> 3)) {
        force_recode = 1;
      }
    }
  }
  return force_recode;
}

void av1_update_reference_frames(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  BufferPool *const pool = cm->buffer_pool;

  // At this point the new frame has been encoded.
  // If any buffer copy / swapping is signaled it should be done here.
  if (cm->frame_type == KEY_FRAME) {
    ref_cnt_fb(pool->frame_bufs, &cm->ref_frame_map[cpi->gld_fb_idx],
               cm->new_fb_idx);
    ref_cnt_fb(pool->frame_bufs, &cm->ref_frame_map[cpi->alt_fb_idx],
               cm->new_fb_idx);
  } else if (av1_preserve_existing_gf(cpi)) {
    // We have decided to preserve the previously existing golden frame as our
    // new ARF frame. However, in the short term in function
    // av1_bitstream.c::get_refresh_mask() we left it in the GF slot and, if
    // we're updating the GF with the current decoded frame, we save it to the
    // ARF slot instead.
    // We now have to update the ARF with the current frame and swap gld_fb_idx
    // and alt_fb_idx so that, overall, we've stored the old GF in the new ARF
    // slot and, if we're updating the GF, the current frame becomes the new GF.
    int tmp;

    ref_cnt_fb(pool->frame_bufs, &cm->ref_frame_map[cpi->alt_fb_idx],
               cm->new_fb_idx);

    tmp = cpi->alt_fb_idx;
    cpi->alt_fb_idx = cpi->gld_fb_idx;
    cpi->gld_fb_idx = tmp;
  } else { /* For non key/golden frames */
    if (cpi->refresh_alt_ref_frame) {
      int arf_idx = cpi->alt_fb_idx;
      if ((cpi->oxcf.pass == 2) && cpi->multi_arf_allowed) {
        const GF_GROUP *const gf_group = &cpi->twopass.gf_group;
        arf_idx = gf_group->arf_update_idx[gf_group->index];
      }

      ref_cnt_fb(pool->frame_bufs, &cm->ref_frame_map[arf_idx], cm->new_fb_idx);
      memcpy(cpi->interp_filter_selected[ALTREF_FRAME],
             cpi->interp_filter_selected[0],
             sizeof(cpi->interp_filter_selected[0]));
    }

    if (cpi->refresh_golden_frame) {
      ref_cnt_fb(pool->frame_bufs, &cm->ref_frame_map[cpi->gld_fb_idx],
                 cm->new_fb_idx);
      if (!cpi->rc.is_src_frame_alt_ref)
        memcpy(cpi->interp_filter_selected[GOLDEN_FRAME],
               cpi->interp_filter_selected[0],
               sizeof(cpi->interp_filter_selected[0]));
      else
        memcpy(cpi->interp_filter_selected[GOLDEN_FRAME],
               cpi->interp_filter_selected[ALTREF_FRAME],
               sizeof(cpi->interp_filter_selected[ALTREF_FRAME]));
    }
  }

  if (cpi->refresh_last_frame) {
    ref_cnt_fb(pool->frame_bufs, &cm->ref_frame_map[cpi->lst_fb_idx],
               cm->new_fb_idx);
    if (!cpi->rc.is_src_frame_alt_ref)
      memcpy(cpi->interp_filter_selected[LAST_FRAME],
             cpi->interp_filter_selected[0],
             sizeof(cpi->interp_filter_selected[0]));
  }
}

static void loopfilter_frame(AV1_COMP *cpi, AV1_COMMON *cm) {
  MACROBLOCKD *xd = &cpi->td.mb.e_mbd;
  struct loopfilter *lf = &cm->lf;
  if (is_lossless_requested(&cpi->oxcf)) {
    lf->filter_level = 0;
  } else {
    struct aom_usec_timer timer;

    aom_clear_system_state();

    aom_usec_timer_start(&timer);

    av1_pick_filter_level(cpi->Source, cpi, cpi->sf.lpf_pick);

    aom_usec_timer_mark(&timer);
    cpi->time_pick_lpf += aom_usec_timer_elapsed(&timer);
  }

  if (lf->filter_level > 0) {
    if (cpi->num_workers > 1)
      av1_loop_filter_frame_mt(cm->frame_to_show, cm, xd->plane,
                               lf->filter_level, 0, 0, cpi->workers,
                               cpi->num_workers, &cpi->lf_row_sync);
    else
      av1_loop_filter_frame(cm->frame_to_show, cm, xd, lf->filter_level, 0, 0);
  }

#if CONFIG_DERING
  if (is_lossless_requested(&cpi->oxcf)) {
    cm->dering_level = 0;
  } else {
    cm->dering_level =
        av1_dering_search(cm->frame_to_show, cpi->Source, cm, xd);
    av1_dering_frame(cm->frame_to_show, cm, xd, cm->dering_level);
  }
#endif  // CONFIG_DERING

#if CONFIG_CLPF
  cm->clpf = 0;
  if (!is_lossless_requested(&cpi->oxcf)) {
    // Test CLPF
    int i, hq = 1;
    uint64_t before, after;
    // TODO(yaowu): investigate per-segment CLPF decision and
    // an optimal threshold, use 80 for now.
    for (i = 0; i < MAX_SEGMENTS; i++)
      hq &= av1_get_qindex(&cm->seg, i, cm->base_qindex) < 80;

    if (!hq) {  // Don't try filter if the entire image is nearly losslessly
                // encoded
#if CLPF_FILTER_ALL_PLANES
      aom_yv12_copy_frame(cm->frame_to_show, &cpi->last_frame_uf);
      before =
          get_sse(cpi->Source->y_buffer, cpi->Source->y_stride,
                  cm->frame_to_show->y_buffer, cm->frame_to_show->y_stride,
                  cpi->Source->y_crop_width, cpi->Source->y_crop_height) +
          get_sse(cpi->Source->u_buffer, cpi->Source->uv_stride,
                  cm->frame_to_show->u_buffer, cm->frame_to_show->uv_stride,
                  cpi->Source->uv_crop_width, cpi->Source->uv_crop_height) +
          get_sse(cpi->Source->v_buffer, cpi->Source->uv_stride,
                  cm->frame_to_show->v_buffer, cm->frame_to_show->uv_stride,
                  cpi->Source->uv_crop_width, cpi->Source->uv_crop_height);
      av1_clpf_frame(cm->frame_to_show, cm, xd);
      after = get_sse(cpi->Source->y_buffer, cpi->Source->y_stride,
                      cm->frame_to_show->y_buffer, cm->frame_to_show->y_stride,
                      cpi->Source->y_crop_width, cpi->Source->y_crop_height) +
              get_sse(cpi->Source->u_buffer, cpi->Source->uv_stride,
                      cm->frame_to_show->u_buffer, cm->frame_to_show->uv_stride,
                      cpi->Source->uv_crop_width, cpi->Source->uv_crop_height) +
              get_sse(cpi->Source->v_buffer, cpi->Source->uv_stride,
                      cm->frame_to_show->v_buffer, cm->frame_to_show->uv_stride,
                      cpi->Source->uv_crop_width, cpi->Source->uv_crop_height);
#else
      aom_yv12_copy_y(cm->frame_to_show, &cpi->last_frame_uf);
      before = get_sse(cpi->Source->y_buffer, cpi->Source->y_stride,
                       cm->frame_to_show->y_buffer, cm->frame_to_show->y_stride,
                       cpi->Source->y_crop_width, cpi->Source->y_crop_height);
      av1_clpf_frame(cm->frame_to_show, cm, xd);
      after = get_sse(cpi->Source->y_buffer, cpi->Source->y_stride,
                      cm->frame_to_show->y_buffer, cm->frame_to_show->y_stride,
                      cpi->Source->y_crop_width, cpi->Source->y_crop_height);
#endif
      if (before < after) {
// No improvement, restore original
#if CLPF_FILTER_ALL_PLANES
        aom_yv12_copy_frame(&cpi->last_frame_uf, cm->frame_to_show);
#else
        aom_yv12_copy_y(&cpi->last_frame_uf, cm->frame_to_show);
#endif
      } else {
        cm->clpf = 1;
      }
    }
  }
#endif

  aom_extend_frame_inner_borders(cm->frame_to_show);
}

static INLINE void alloc_frame_mvs(const AV1_COMMON *cm, int buffer_idx) {
  RefCntBuffer *const new_fb_ptr = &cm->buffer_pool->frame_bufs[buffer_idx];
  if (new_fb_ptr->mvs == NULL || new_fb_ptr->mi_rows < cm->mi_rows ||
      new_fb_ptr->mi_cols < cm->mi_cols) {
    aom_free(new_fb_ptr->mvs);
    new_fb_ptr->mvs = (MV_REF *)aom_calloc(cm->mi_rows * cm->mi_cols,
                                           sizeof(*new_fb_ptr->mvs));
    new_fb_ptr->mi_rows = cm->mi_rows;
    new_fb_ptr->mi_cols = cm->mi_cols;
  }
}

void av1_scale_references(AV1_COMP *cpi) {
  AV1_COMMON *cm = &cpi->common;
  MV_REFERENCE_FRAME ref_frame;
  const AOM_REFFRAME ref_mask[3] = { AOM_LAST_FLAG, AOM_GOLD_FLAG,
                                     AOM_ALT_FLAG };

  for (ref_frame = LAST_FRAME; ref_frame <= ALTREF_FRAME; ++ref_frame) {
    // Need to convert from AOM_REFFRAME to index into ref_mask (subtract 1).
    if (cpi->ref_frame_flags & ref_mask[ref_frame - 1]) {
      BufferPool *const pool = cm->buffer_pool;
      const YV12_BUFFER_CONFIG *const ref =
          get_ref_frame_buffer(cpi, ref_frame);

      if (ref == NULL) {
        cpi->scaled_ref_idx[ref_frame - 1] = INVALID_IDX;
        continue;
      }

#if CONFIG_AOM_HIGHBITDEPTH
      if (ref->y_crop_width != cm->width || ref->y_crop_height != cm->height) {
        RefCntBuffer *new_fb_ptr = NULL;
        int force_scaling = 0;
        int new_fb = cpi->scaled_ref_idx[ref_frame - 1];
        if (new_fb == INVALID_IDX) {
          new_fb = get_free_fb(cm);
          force_scaling = 1;
        }
        if (new_fb == INVALID_IDX) return;
        new_fb_ptr = &pool->frame_bufs[new_fb];
        if (force_scaling || new_fb_ptr->buf.y_crop_width != cm->width ||
            new_fb_ptr->buf.y_crop_height != cm->height) {
          aom_realloc_frame_buffer(&new_fb_ptr->buf, cm->width, cm->height,
                                   cm->subsampling_x, cm->subsampling_y,
                                   cm->use_highbitdepth, AOM_BORDER_IN_PIXELS,
                                   cm->byte_alignment, NULL, NULL, NULL);
          scale_and_extend_frame(ref, &new_fb_ptr->buf, (int)cm->bit_depth);
          cpi->scaled_ref_idx[ref_frame - 1] = new_fb;
          alloc_frame_mvs(cm, new_fb);
        }
#else
      if (ref->y_crop_width != cm->width || ref->y_crop_height != cm->height) {
        RefCntBuffer *new_fb_ptr = NULL;
        int force_scaling = 0;
        int new_fb = cpi->scaled_ref_idx[ref_frame - 1];
        if (new_fb == INVALID_IDX) {
          new_fb = get_free_fb(cm);
          force_scaling = 1;
        }
        if (new_fb == INVALID_IDX) return;
        new_fb_ptr = &pool->frame_bufs[new_fb];
        if (force_scaling || new_fb_ptr->buf.y_crop_width != cm->width ||
            new_fb_ptr->buf.y_crop_height != cm->height) {
          aom_realloc_frame_buffer(&new_fb_ptr->buf, cm->width, cm->height,
                                   cm->subsampling_x, cm->subsampling_y,
                                   AOM_BORDER_IN_PIXELS, cm->byte_alignment,
                                   NULL, NULL, NULL);
          scale_and_extend_frame(ref, &new_fb_ptr->buf);
          cpi->scaled_ref_idx[ref_frame - 1] = new_fb;
          alloc_frame_mvs(cm, new_fb);
        }
#endif  // CONFIG_AOM_HIGHBITDEPTH
      } else {
        const int buf_idx = get_ref_frame_buf_idx(cpi, ref_frame);
        RefCntBuffer *const buf = &pool->frame_bufs[buf_idx];
        buf->buf.y_crop_width = ref->y_crop_width;
        buf->buf.y_crop_height = ref->y_crop_height;
        cpi->scaled_ref_idx[ref_frame - 1] = buf_idx;
        ++buf->ref_count;
      }
    } else {
      if (cpi->oxcf.pass != 0) cpi->scaled_ref_idx[ref_frame - 1] = INVALID_IDX;
    }
  }
}

static void release_scaled_references(AV1_COMP *cpi) {
  AV1_COMMON *cm = &cpi->common;
  int i;
  if (cpi->oxcf.pass == 0) {
    // Only release scaled references under certain conditions:
    // if reference will be updated, or if scaled reference has same resolution.
    int refresh[3];
    refresh[0] = (cpi->refresh_last_frame) ? 1 : 0;
    refresh[1] = (cpi->refresh_golden_frame) ? 1 : 0;
    refresh[2] = (cpi->refresh_alt_ref_frame) ? 1 : 0;
    for (i = LAST_FRAME; i <= ALTREF_FRAME; ++i) {
      const int idx = cpi->scaled_ref_idx[i - 1];
      RefCntBuffer *const buf =
          idx != INVALID_IDX ? &cm->buffer_pool->frame_bufs[idx] : NULL;
      const YV12_BUFFER_CONFIG *const ref = get_ref_frame_buffer(cpi, i);
      if (buf != NULL &&
          (refresh[i - 1] || (buf->buf.y_crop_width == ref->y_crop_width &&
                              buf->buf.y_crop_height == ref->y_crop_height))) {
        --buf->ref_count;
        cpi->scaled_ref_idx[i - 1] = INVALID_IDX;
      }
    }
  } else {
    for (i = 0; i < MAX_REF_FRAMES; ++i) {
      const int idx = cpi->scaled_ref_idx[i];
      RefCntBuffer *const buf =
          idx != INVALID_IDX ? &cm->buffer_pool->frame_bufs[idx] : NULL;
      if (buf != NULL) {
        --buf->ref_count;
        cpi->scaled_ref_idx[i] = INVALID_IDX;
      }
    }
  }
}

static void full_to_model_count(unsigned int *model_count,
                                unsigned int *full_count) {
  int n;
  model_count[ZERO_TOKEN] = full_count[ZERO_TOKEN];
  model_count[ONE_TOKEN] = full_count[ONE_TOKEN];
  model_count[TWO_TOKEN] = full_count[TWO_TOKEN];
  for (n = THREE_TOKEN; n < EOB_TOKEN; ++n)
    model_count[TWO_TOKEN] += full_count[n];
  model_count[EOB_MODEL_TOKEN] = full_count[EOB_TOKEN];
}

static void full_to_model_counts(av1_coeff_count_model *model_count,
                                 av1_coeff_count *full_count) {
  int i, j, k, l;

  for (i = 0; i < PLANE_TYPES; ++i)
    for (j = 0; j < REF_TYPES; ++j)
      for (k = 0; k < COEF_BANDS; ++k)
        for (l = 0; l < BAND_COEFF_CONTEXTS(k); ++l)
          full_to_model_count(model_count[i][j][k][l], full_count[i][j][k][l]);
}

#if 0 && CONFIG_INTERNAL_STATS
static void output_frame_level_debug_stats(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  FILE *const f = fopen("tmp.stt", cm->current_video_frame ? "a" : "w");
  int64_t recon_err;

  aom_clear_system_state();

  recon_err = aom_get_y_sse(cpi->Source, get_frame_new_buffer(cm));

  if (cpi->twopass.total_left_stats.coded_error != 0.0)
    fprintf(f, "%10u %dx%d  %10d %10d %d %d %10d %10d %10d %10d"
       "%10"PRId64" %10"PRId64" %5d %5d %10"PRId64" "
       "%10"PRId64" %10"PRId64" %10d "
       "%7.2lf %7.2lf %7.2lf %7.2lf %7.2lf"
        "%6d %6d %5d %5d %5d "
        "%10"PRId64" %10.3lf"
        "%10lf %8u %10"PRId64" %10d %10d %10d\n",
        cpi->common.current_video_frame,
        cm->width, cm->height,
        cpi->td.rd_counts.m_search_count,
        cpi->td.rd_counts.ex_search_count,
        cpi->rc.source_alt_ref_pending,
        cpi->rc.source_alt_ref_active,
        cpi->rc.this_frame_target,
        cpi->rc.projected_frame_size,
        cpi->rc.projected_frame_size / cpi->common.MBs,
        (cpi->rc.projected_frame_size - cpi->rc.this_frame_target),
        cpi->rc.vbr_bits_off_target,
        cpi->rc.vbr_bits_off_target_fast,
        cpi->twopass.extend_minq,
        cpi->twopass.extend_minq_fast,
        cpi->rc.total_target_vs_actual,
        (cpi->rc.starting_buffer_level - cpi->rc.bits_off_target),
        cpi->rc.total_actual_bits, cm->base_qindex,
        av1_convert_qindex_to_q(cm->base_qindex, cm->bit_depth),
        (double)av1_dc_quant(cm->base_qindex, 0, cm->bit_depth) / 4.0,
        av1_convert_qindex_to_q(cpi->twopass.active_worst_quality,
                                cm->bit_depth),
        cpi->rc.avg_q,
        av1_convert_qindex_to_q(cpi->oxcf.cq_level, cm->bit_depth),
        cpi->refresh_last_frame, cpi->refresh_golden_frame,
        cpi->refresh_alt_ref_frame, cm->frame_type, cpi->rc.gfu_boost,
        cpi->twopass.bits_left,
        cpi->twopass.total_left_stats.coded_error,
        cpi->twopass.bits_left /
            (1 + cpi->twopass.total_left_stats.coded_error),
        cpi->tot_recode_hits, recon_err, cpi->rc.kf_boost,
        cpi->twopass.kf_zeromotion_pct,
        cpi->twopass.fr_content_type);

  fclose(f);

  if (0) {
    FILE *const fmodes = fopen("Modes.stt", "a");
    int i;

    fprintf(fmodes, "%6d:%1d:%1d:%1d ", cpi->common.current_video_frame,
            cm->frame_type, cpi->refresh_golden_frame,
            cpi->refresh_alt_ref_frame);

    for (i = 0; i < MAX_MODES; ++i)
      fprintf(fmodes, "%5d ", cpi->mode_chosen_counts[i]);

    fprintf(fmodes, "\n");

    fclose(fmodes);
  }
}
#endif

static void set_mv_search_params(AV1_COMP *cpi) {
  const AV1_COMMON *const cm = &cpi->common;
  const unsigned int max_mv_def = AOMMIN(cm->width, cm->height);

  // Default based on max resolution.
  cpi->mv_step_param = av1_init_search_range(max_mv_def);

  if (cpi->sf.mv.auto_mv_step_size) {
    if (frame_is_intra_only(cm)) {
      // Initialize max_mv_magnitude for use in the first INTER frame
      // after a key/intra-only frame.
      cpi->max_mv_magnitude = max_mv_def;
    } else {
      if (cm->show_frame) {
        // Allow mv_steps to correspond to twice the max mv magnitude found
        // in the previous frame, capped by the default max_mv_magnitude based
        // on resolution.
        cpi->mv_step_param = av1_init_search_range(
            AOMMIN(max_mv_def, 2 * cpi->max_mv_magnitude));
      }
      cpi->max_mv_magnitude = 0;
    }
  }
}

static void set_size_independent_vars(AV1_COMP *cpi) {
  av1_set_speed_features_framesize_independent(cpi);
  av1_set_rd_speed_thresholds(cpi);
  av1_set_rd_speed_thresholds_sub8x8(cpi);
  cpi->common.interp_filter = cpi->sf.default_interp_filter;
}

static void set_size_dependent_vars(AV1_COMP *cpi, int *q, int *bottom_index,
                                    int *top_index) {
  AV1_COMMON *const cm = &cpi->common;
  const AV1EncoderConfig *const oxcf = &cpi->oxcf;

  // Setup variables that depend on the dimensions of the frame.
  av1_set_speed_features_framesize_dependent(cpi);

  // Decide q and q bounds.
  *q = av1_rc_pick_q_and_bounds(cpi, bottom_index, top_index);

  if (!frame_is_intra_only(cm)) {
    av1_set_high_precision_mv(cpi, (*q) < HIGH_PRECISION_MV_QTHRESH);
  }

  // Configure experimental use of segmentation for enhanced coding of
  // static regions if indicated.
  // Only allowed in the second pass of a two pass encode, as it requires
  // lagged coding, and if the relevant speed feature flag is set.
  if (oxcf->pass == 2 && cpi->sf.static_segmentation)
    configure_static_seg_features(cpi);
}

static void init_motion_estimation(AV1_COMP *cpi) {
  int y_stride = cpi->scaled_source.y_stride;

  if (cpi->sf.mv.search_method == NSTEP) {
    av1_init3smotion_compensation(&cpi->ss_cfg, y_stride);
  } else if (cpi->sf.mv.search_method == DIAMOND) {
    av1_init_dsmotion_compensation(&cpi->ss_cfg, y_stride);
  }
}

static void set_frame_size(AV1_COMP *cpi) {
  int ref_frame;
  AV1_COMMON *const cm = &cpi->common;
  AV1EncoderConfig *const oxcf = &cpi->oxcf;
  MACROBLOCKD *const xd = &cpi->td.mb.e_mbd;

  if (oxcf->pass == 2 && oxcf->rc_mode == AOM_VBR &&
      ((oxcf->resize_mode == RESIZE_FIXED && cm->current_video_frame == 0) ||
       (oxcf->resize_mode == RESIZE_DYNAMIC && cpi->resize_pending))) {
    av1_calculate_coded_size(cpi, &oxcf->scaled_frame_width,
                             &oxcf->scaled_frame_height);

    // There has been a change in frame size.
    av1_set_size_literal(cpi, oxcf->scaled_frame_width,
                         oxcf->scaled_frame_height);
  }

  if (oxcf->pass == 0 && oxcf->rc_mode == AOM_CBR &&
      oxcf->resize_mode == RESIZE_DYNAMIC) {
    if (cpi->resize_pending == 1) {
      oxcf->scaled_frame_width =
          (cm->width * cpi->resize_scale_num) / cpi->resize_scale_den;
      oxcf->scaled_frame_height =
          (cm->height * cpi->resize_scale_num) / cpi->resize_scale_den;
    } else if (cpi->resize_pending == -1) {
      // Go back up to original size.
      oxcf->scaled_frame_width = oxcf->width;
      oxcf->scaled_frame_height = oxcf->height;
    }
    if (cpi->resize_pending != 0) {
      // There has been a change in frame size.
      av1_set_size_literal(cpi, oxcf->scaled_frame_width,
                           oxcf->scaled_frame_height);

      // TODO(agrange) Scale cpi->max_mv_magnitude if frame-size has changed.
      set_mv_search_params(cpi);
    }
  }

  if (oxcf->pass == 2) {
    av1_set_target_rate(cpi);
  }

  alloc_frame_mvs(cm, cm->new_fb_idx);

  // Reset the frame pointers to the current frame size.
  aom_realloc_frame_buffer(get_frame_new_buffer(cm), cm->width, cm->height,
                           cm->subsampling_x, cm->subsampling_y,
#if CONFIG_AOM_HIGHBITDEPTH
                           cm->use_highbitdepth,
#endif
                           AOM_BORDER_IN_PIXELS, cm->byte_alignment, NULL, NULL,
                           NULL);

  alloc_util_frame_buffers(cpi);
  init_motion_estimation(cpi);

  for (ref_frame = LAST_FRAME; ref_frame <= ALTREF_FRAME; ++ref_frame) {
    RefBuffer *const ref_buf = &cm->frame_refs[ref_frame - 1];
    const int buf_idx = get_ref_frame_buf_idx(cpi, ref_frame);

    ref_buf->idx = buf_idx;

    if (buf_idx != INVALID_IDX) {
      YV12_BUFFER_CONFIG *const buf = &cm->buffer_pool->frame_bufs[buf_idx].buf;
      ref_buf->buf = buf;
#if CONFIG_AOM_HIGHBITDEPTH
      av1_setup_scale_factors_for_frame(
          &ref_buf->sf, buf->y_crop_width, buf->y_crop_height, cm->width,
          cm->height, (buf->flags & YV12_FLAG_HIGHBITDEPTH) ? 1 : 0);
#else
      av1_setup_scale_factors_for_frame(&ref_buf->sf, buf->y_crop_width,
                                        buf->y_crop_height, cm->width,
                                        cm->height);
#endif  // CONFIG_AOM_HIGHBITDEPTH
      if (av1_is_scaled(&ref_buf->sf)) aom_extend_frame_borders(buf);
    } else {
      ref_buf->buf = NULL;
    }
  }

  set_ref_ptrs(cm, xd, LAST_FRAME, LAST_FRAME);
}

static void encode_without_recode_loop(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  int q = 0, bottom_index = 0, top_index = 0;  // Dummy variables.

  aom_clear_system_state();

  set_frame_size(cpi);

  // For 1 pass CBR under dynamic resize mode: use faster scaling for source.
  // Only for 2x2 scaling for now.
  if (cpi->oxcf.pass == 0 && cpi->oxcf.rc_mode == AOM_CBR &&
      cpi->oxcf.resize_mode == RESIZE_DYNAMIC &&
      cpi->un_scaled_source->y_width == (cm->width << 1) &&
      cpi->un_scaled_source->y_height == (cm->height << 1)) {
    cpi->Source = av1_scale_if_required_fast(cm, cpi->un_scaled_source,
                                             &cpi->scaled_source);
    if (cpi->unscaled_last_source != NULL)
      cpi->Last_Source = av1_scale_if_required_fast(
          cm, cpi->unscaled_last_source, &cpi->scaled_last_source);
  } else {
    cpi->Source =
        av1_scale_if_required(cm, cpi->un_scaled_source, &cpi->scaled_source);
    if (cpi->unscaled_last_source != NULL)
      cpi->Last_Source = av1_scale_if_required(cm, cpi->unscaled_last_source,
                                               &cpi->scaled_last_source);
  }

  if (frame_is_intra_only(cm) == 0) {
    av1_scale_references(cpi);
  }

  set_size_independent_vars(cpi);
  set_size_dependent_vars(cpi, &q, &bottom_index, &top_index);

  av1_set_quantizer(cm, q);
  av1_set_variance_partition_thresholds(cpi, q);

  setup_frame(cpi);

  suppress_active_map(cpi);
  // Variance adaptive and in frame q adjustment experiments are mutually
  // exclusive.
  if (cpi->oxcf.aq_mode == VARIANCE_AQ) {
    av1_vaq_frame_setup(cpi);
  } else if (cpi->oxcf.aq_mode == COMPLEXITY_AQ) {
    av1_setup_in_frame_q_adj(cpi);
  } else if (cpi->oxcf.aq_mode == CYCLIC_REFRESH_AQ) {
    av1_cyclic_refresh_setup(cpi);
  }
  apply_active_map(cpi);

  // transform / motion compensation build reconstruction frame
  av1_encode_frame(cpi);

  // Update some stats from cyclic refresh, and check if we should not update
  // golden reference, for 1 pass CBR.
  if (cpi->oxcf.aq_mode == CYCLIC_REFRESH_AQ && cm->frame_type != KEY_FRAME &&
      (cpi->oxcf.pass == 0 && cpi->oxcf.rc_mode == AOM_CBR))
    av1_cyclic_refresh_check_golden_update(cpi);

  // Update the skip mb flag probabilities based on the distribution
  // seen in the last encoder iteration.
  // update_base_skip_probs(cpi);
  aom_clear_system_state();
}

static void encode_with_recode_loop(AV1_COMP *cpi, size_t *size,
                                    uint8_t *dest) {
  AV1_COMMON *const cm = &cpi->common;
  RATE_CONTROL *const rc = &cpi->rc;
  int bottom_index, top_index;
  int loop_count = 0;
  int loop_at_this_size = 0;
  int loop = 0;
  int overshoot_seen = 0;
  int undershoot_seen = 0;
  int frame_over_shoot_limit;
  int frame_under_shoot_limit;
  int q = 0, q_low = 0, q_high = 0;

  set_size_independent_vars(cpi);

  do {
    aom_clear_system_state();

    set_frame_size(cpi);

    if (loop_count == 0 || cpi->resize_pending != 0) {
      set_size_dependent_vars(cpi, &q, &bottom_index, &top_index);

      // TODO(agrange) Scale cpi->max_mv_magnitude if frame-size has changed.
      set_mv_search_params(cpi);

      // Reset the loop state for new frame size.
      overshoot_seen = 0;
      undershoot_seen = 0;

      // Reconfiguration for change in frame size has concluded.
      cpi->resize_pending = 0;

      q_low = bottom_index;
      q_high = top_index;

      loop_at_this_size = 0;
    }

    // Decide frame size bounds first time through.
    if (loop_count == 0) {
      av1_rc_compute_frame_size_bounds(cpi, rc->this_frame_target,
                                       &frame_under_shoot_limit,
                                       &frame_over_shoot_limit);
    }

    cpi->Source =
        av1_scale_if_required(cm, cpi->un_scaled_source, &cpi->scaled_source);

    if (cpi->unscaled_last_source != NULL)
      cpi->Last_Source = av1_scale_if_required(cm, cpi->unscaled_last_source,
                                               &cpi->scaled_last_source);

    if (frame_is_intra_only(cm) == 0) {
      if (loop_count > 0) {
        release_scaled_references(cpi);
      }
      av1_scale_references(cpi);
    }

    av1_set_quantizer(cm, q);

    if (loop_count == 0) setup_frame(cpi);

    // Variance adaptive and in frame q adjustment experiments are mutually
    // exclusive.
    if (cpi->oxcf.aq_mode == VARIANCE_AQ) {
      av1_vaq_frame_setup(cpi);
    } else if (cpi->oxcf.aq_mode == COMPLEXITY_AQ) {
      av1_setup_in_frame_q_adj(cpi);
    }

    // transform / motion compensation build reconstruction frame
    av1_encode_frame(cpi);

    // Update the skip mb flag probabilities based on the distribution
    // seen in the last encoder iteration.
    // update_base_skip_probs(cpi);

    aom_clear_system_state();

    // Dummy pack of the bitstream using up to date stats to get an
    // accurate estimate of output frame size to determine if we need
    // to recode.
    if (cpi->sf.recode_loop >= ALLOW_RECODE_KFARFGF) {
      save_coding_context(cpi);
      av1_pack_bitstream(cpi, dest, size);

      rc->projected_frame_size = (int)(*size) << 3;
      restore_coding_context(cpi);

      if (frame_over_shoot_limit == 0) frame_over_shoot_limit = 1;
    }

    if (cpi->oxcf.rc_mode == AOM_Q) {
      loop = 0;
    } else {
      if ((cm->frame_type == KEY_FRAME) && rc->this_key_frame_forced &&
          (rc->projected_frame_size < rc->max_frame_bandwidth)) {
        int last_q = q;
        int64_t kf_err;

        int64_t high_err_target = cpi->ambient_err;
        int64_t low_err_target = cpi->ambient_err >> 1;

#if CONFIG_AOM_HIGHBITDEPTH
        if (cm->use_highbitdepth) {
          kf_err = aom_highbd_get_y_sse(cpi->Source, get_frame_new_buffer(cm));
        } else {
          kf_err = aom_get_y_sse(cpi->Source, get_frame_new_buffer(cm));
        }
#else
        kf_err = aom_get_y_sse(cpi->Source, get_frame_new_buffer(cm));
#endif  // CONFIG_AOM_HIGHBITDEPTH

        // Prevent possible divide by zero error below for perfect KF
        kf_err += !kf_err;

        // The key frame is not good enough or we can afford
        // to make it better without undue risk of popping.
        if ((kf_err > high_err_target &&
             rc->projected_frame_size <= frame_over_shoot_limit) ||
            (kf_err > low_err_target &&
             rc->projected_frame_size <= frame_under_shoot_limit)) {
          // Lower q_high
          q_high = q > q_low ? q - 1 : q_low;

          // Adjust Q
          q = (int)((q * high_err_target) / kf_err);
          q = AOMMIN(q, (q_high + q_low) >> 1);
        } else if (kf_err < low_err_target &&
                   rc->projected_frame_size >= frame_under_shoot_limit) {
          // The key frame is much better than the previous frame
          // Raise q_low
          q_low = q < q_high ? q + 1 : q_high;

          // Adjust Q
          q = (int)((q * low_err_target) / kf_err);
          q = AOMMIN(q, (q_high + q_low + 1) >> 1);
        }

        // Clamp Q to upper and lower limits:
        q = clamp(q, q_low, q_high);

        loop = q != last_q;
      } else if (recode_loop_test(cpi, frame_over_shoot_limit,
                                  frame_under_shoot_limit, q,
                                  AOMMAX(q_high, top_index), bottom_index)) {
        // Is the projected frame size out of range and are we allowed
        // to attempt to recode.
        int last_q = q;
        int retries = 0;

        if (cpi->resize_pending == 1) {
          // Change in frame size so go back around the recode loop.
          cpi->rc.frame_size_selector =
              SCALE_STEP1 - cpi->rc.frame_size_selector;
          cpi->rc.next_frame_size_selector = cpi->rc.frame_size_selector;

#if CONFIG_INTERNAL_STATS
          ++cpi->tot_recode_hits;
#endif
          ++loop_count;
          loop = 1;
          continue;
        }

        // Frame size out of permitted range:
        // Update correction factor & compute new Q to try...

        // Frame is too large
        if (rc->projected_frame_size > rc->this_frame_target) {
          // Special case if the projected size is > the max allowed.
          if (rc->projected_frame_size >= rc->max_frame_bandwidth)
            q_high = rc->worst_quality;

          // Raise Qlow as to at least the current value
          q_low = q < q_high ? q + 1 : q_high;

          if (undershoot_seen || loop_at_this_size > 1) {
            // Update rate_correction_factor unless
            av1_rc_update_rate_correction_factors(cpi);

            q = (q_high + q_low + 1) / 2;
          } else {
            // Update rate_correction_factor unless
            av1_rc_update_rate_correction_factors(cpi);

            q = av1_rc_regulate_q(cpi, rc->this_frame_target, bottom_index,
                                  AOMMAX(q_high, top_index));

            while (q < q_low && retries < 10) {
              av1_rc_update_rate_correction_factors(cpi);
              q = av1_rc_regulate_q(cpi, rc->this_frame_target, bottom_index,
                                    AOMMAX(q_high, top_index));
              retries++;
            }
          }

          overshoot_seen = 1;
        } else {
          // Frame is too small
          q_high = q > q_low ? q - 1 : q_low;

          if (overshoot_seen || loop_at_this_size > 1) {
            av1_rc_update_rate_correction_factors(cpi);
            q = (q_high + q_low) / 2;
          } else {
            av1_rc_update_rate_correction_factors(cpi);
            q = av1_rc_regulate_q(cpi, rc->this_frame_target, bottom_index,
                                  top_index);
            // Special case reset for qlow for constrained quality.
            // This should only trigger where there is very substantial
            // undershoot on a frame and the auto cq level is above
            // the user passsed in value.
            if (cpi->oxcf.rc_mode == AOM_CQ && q < q_low) {
              q_low = q;
            }

            while (q > q_high && retries < 10) {
              av1_rc_update_rate_correction_factors(cpi);
              q = av1_rc_regulate_q(cpi, rc->this_frame_target, bottom_index,
                                    top_index);
              retries++;
            }
          }

          undershoot_seen = 1;
        }

        // Clamp Q to upper and lower limits:
        q = clamp(q, q_low, q_high);

        loop = (q != last_q);
      } else {
        loop = 0;
      }
    }

    // Special case for overlay frame.
    if (rc->is_src_frame_alt_ref &&
        rc->projected_frame_size < rc->max_frame_bandwidth)
      loop = 0;

    if (loop) {
      ++loop_count;
      ++loop_at_this_size;

#if CONFIG_INTERNAL_STATS
      ++cpi->tot_recode_hits;
#endif
    }
  } while (loop);
}

static int get_ref_frame_flags(const AV1_COMP *cpi) {
  const int *const map = cpi->common.ref_frame_map;
  const int gold_is_last = map[cpi->gld_fb_idx] == map[cpi->lst_fb_idx];
  const int alt_is_last = map[cpi->alt_fb_idx] == map[cpi->lst_fb_idx];
  const int gold_is_alt = map[cpi->gld_fb_idx] == map[cpi->alt_fb_idx];
  int flags = AOM_ALT_FLAG | AOM_GOLD_FLAG | AOM_LAST_FLAG;

  if (gold_is_last) flags &= ~AOM_GOLD_FLAG;

  if (cpi->rc.frames_till_gf_update_due == INT_MAX) flags &= ~AOM_GOLD_FLAG;

  if (alt_is_last) flags &= ~AOM_ALT_FLAG;

  if (gold_is_alt) flags &= ~AOM_ALT_FLAG;

  return flags;
}

static void set_ext_overrides(AV1_COMP *cpi) {
  // Overrides the defaults with the externally supplied values with
  // av1_update_reference() and av1_update_entropy() calls
  // Note: The overrides are valid only for the next frame passed
  // to encode_frame_to_data_rate() function
  if (cpi->ext_refresh_frame_context_pending) {
    cpi->common.refresh_frame_context = cpi->ext_refresh_frame_context;
    cpi->ext_refresh_frame_context_pending = 0;
  }
  if (cpi->ext_refresh_frame_flags_pending) {
    cpi->refresh_last_frame = cpi->ext_refresh_last_frame;
    cpi->refresh_golden_frame = cpi->ext_refresh_golden_frame;
    cpi->refresh_alt_ref_frame = cpi->ext_refresh_alt_ref_frame;
    cpi->ext_refresh_frame_flags_pending = 0;
  }
}

YV12_BUFFER_CONFIG *av1_scale_if_required_fast(AV1_COMMON *cm,
                                               YV12_BUFFER_CONFIG *unscaled,
                                               YV12_BUFFER_CONFIG *scaled) {
  if (cm->mi_cols * MI_SIZE != unscaled->y_width ||
      cm->mi_rows * MI_SIZE != unscaled->y_height) {
    // For 2x2 scaling down.
    aom_scale_frame(unscaled, scaled, unscaled->y_buffer, 9, 2, 1, 2, 1, 0);
    aom_extend_frame_borders(scaled);
    return scaled;
  } else {
    return unscaled;
  }
}

YV12_BUFFER_CONFIG *av1_scale_if_required(AV1_COMMON *cm,
                                          YV12_BUFFER_CONFIG *unscaled,
                                          YV12_BUFFER_CONFIG *scaled) {
  if (cm->mi_cols * MI_SIZE != unscaled->y_width ||
      cm->mi_rows * MI_SIZE != unscaled->y_height) {
#if CONFIG_AOM_HIGHBITDEPTH
    scale_and_extend_frame_nonnormative(unscaled, scaled, (int)cm->bit_depth);
#else
    scale_and_extend_frame_nonnormative(unscaled, scaled);
#endif  // CONFIG_AOM_HIGHBITDEPTH
    return scaled;
  } else {
    return unscaled;
  }
}

static void set_arf_sign_bias(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  int arf_sign_bias;

  if ((cpi->oxcf.pass == 2) && cpi->multi_arf_allowed) {
    const GF_GROUP *const gf_group = &cpi->twopass.gf_group;
    arf_sign_bias = cpi->rc.source_alt_ref_active &&
                    (!cpi->refresh_alt_ref_frame ||
                     (gf_group->rf_level[gf_group->index] == GF_ARF_LOW));
  } else {
    arf_sign_bias =
        (cpi->rc.source_alt_ref_active && !cpi->refresh_alt_ref_frame);
  }
  cm->ref_frame_sign_bias[ALTREF_FRAME] = arf_sign_bias;
}

static int setup_interp_filter_search_mask(AV1_COMP *cpi) {
  InterpFilter ifilter;
  int ref_total[MAX_REF_FRAMES] = { 0 };
  MV_REFERENCE_FRAME ref;
  int mask = 0;
  if (cpi->common.last_frame_type == KEY_FRAME || cpi->refresh_alt_ref_frame)
    return mask;
  for (ref = LAST_FRAME; ref <= ALTREF_FRAME; ++ref)
    for (ifilter = EIGHTTAP; ifilter <= EIGHTTAP_SHARP; ++ifilter)
      ref_total[ref] += cpi->interp_filter_selected[ref][ifilter];

  for (ifilter = EIGHTTAP; ifilter <= EIGHTTAP_SHARP; ++ifilter) {
    if ((ref_total[LAST_FRAME] &&
         cpi->interp_filter_selected[LAST_FRAME][ifilter] == 0) &&
        (ref_total[GOLDEN_FRAME] == 0 ||
         cpi->interp_filter_selected[GOLDEN_FRAME][ifilter] * 50 <
             ref_total[GOLDEN_FRAME]) &&
        (ref_total[ALTREF_FRAME] == 0 ||
         cpi->interp_filter_selected[ALTREF_FRAME][ifilter] * 50 <
             ref_total[ALTREF_FRAME]))
      mask |= 1 << ifilter;
  }
  return mask;
}

static void encode_frame_to_data_rate(AV1_COMP *cpi, size_t *size,
                                      uint8_t *dest,
                                      unsigned int *frame_flags) {
  AV1_COMMON *const cm = &cpi->common;
  const AV1EncoderConfig *const oxcf = &cpi->oxcf;
  struct segmentation *const seg = &cm->seg;
  TX_SIZE t;

  set_ext_overrides(cpi);
  aom_clear_system_state();

  // Set the arf sign bias for this frame.
  set_arf_sign_bias(cpi);

  // Set default state for segment based loop filter update flags.
  cm->lf.mode_ref_delta_update = 0;

  if (cpi->oxcf.pass == 2 && cpi->sf.adaptive_interp_filter_search)
    cpi->sf.interp_filter_search_mask = setup_interp_filter_search_mask(cpi);

  // Set various flags etc to special state if it is a key frame.
  if (frame_is_intra_only(cm)) {
    // Reset the loop filter deltas and segmentation map.
    av1_reset_segment_features(cm);

    // If segmentation is enabled force a map update for key frames.
    if (seg->enabled) {
      seg->update_map = 1;
      seg->update_data = 1;
    }

    // The alternate reference frame cannot be active for a key frame.
    cpi->rc.source_alt_ref_active = 0;

    cm->error_resilient_mode = oxcf->error_resilient_mode;

    // By default, encoder assumes decoder can use prev_mi.
    if (cm->error_resilient_mode) {
      cm->reset_frame_context = RESET_FRAME_CONTEXT_NONE;
      cm->refresh_frame_context = REFRESH_FRAME_CONTEXT_OFF;
    } else if (cm->intra_only) {
      // Only reset the current context.
      cm->reset_frame_context = RESET_FRAME_CONTEXT_CURRENT;
    }
  }

  // For 1 pass CBR, check if we are dropping this frame.
  // Never drop on key frame.
  if (oxcf->pass == 0 && oxcf->rc_mode == AOM_CBR &&
      cm->frame_type != KEY_FRAME) {
    if (av1_rc_drop_frame(cpi)) {
      av1_rc_postencode_update_drop_frame(cpi);
      ++cm->current_video_frame;
      return;
    }
  }

  aom_clear_system_state();

#if CONFIG_INTERNAL_STATS
  memset(cpi->mode_chosen_counts, 0,
         MAX_MODES * sizeof(*cpi->mode_chosen_counts));
#endif

  if (cpi->sf.recode_loop == DISALLOW_RECODE) {
    encode_without_recode_loop(cpi);
  } else {
    encode_with_recode_loop(cpi, size, dest);
  }

#ifdef OUTPUT_YUV_SKINMAP
  if (cpi->common.current_video_frame > 1) {
    av1_compute_skin_map(cpi, yuv_skinmap_file);
  }
#endif

  // Special case code to reduce pulsing when key frames are forced at a
  // fixed interval. Note the reconstruction error if it is the frame before
  // the force key frame
  if (cpi->rc.next_key_frame_forced && cpi->rc.frames_to_key == 1) {
#if CONFIG_AOM_HIGHBITDEPTH
    if (cm->use_highbitdepth) {
      cpi->ambient_err =
          aom_highbd_get_y_sse(cpi->Source, get_frame_new_buffer(cm));
    } else {
      cpi->ambient_err = aom_get_y_sse(cpi->Source, get_frame_new_buffer(cm));
    }
#else
    cpi->ambient_err = aom_get_y_sse(cpi->Source, get_frame_new_buffer(cm));
#endif  // CONFIG_AOM_HIGHBITDEPTH
  }

  // If the encoder forced a KEY_FRAME decision
  if (cm->frame_type == KEY_FRAME) cpi->refresh_last_frame = 1;

  cm->frame_to_show = get_frame_new_buffer(cm);
  cm->frame_to_show->color_space = cm->color_space;
  cm->frame_to_show->color_range = cm->color_range;
  cm->frame_to_show->render_width = cm->render_width;
  cm->frame_to_show->render_height = cm->render_height;

  // Pick the loop filter level for the frame.
  loopfilter_frame(cpi, cm);

  // build the bitstream
  av1_pack_bitstream(cpi, dest, size);

  if (cm->seg.update_map) update_reference_segmentation_map(cpi);

  if (frame_is_intra_only(cm) == 0) {
    release_scaled_references(cpi);
  }
  av1_update_reference_frames(cpi);

  for (t = TX_4X4; t <= TX_32X32; t++)
    full_to_model_counts(cpi->td.counts->coef[t],
                         cpi->td.rd_counts.coef_counts[t]);

  if (cm->refresh_frame_context == REFRESH_FRAME_CONTEXT_BACKWARD) {
    av1_adapt_coef_probs(cm);
#if CONFIG_MISC_FIXES
    av1_adapt_intra_frame_probs(cm);
#else
    if (!frame_is_intra_only(cm)) av1_adapt_intra_frame_probs(cm);
#endif
  }

  if (!frame_is_intra_only(cm)) {
    if (cm->refresh_frame_context == REFRESH_FRAME_CONTEXT_BACKWARD) {
      av1_adapt_inter_frame_probs(cm);
      av1_adapt_mv_probs(cm, cm->allow_high_precision_mv);
    }
  }

  if (cpi->refresh_golden_frame == 1)
    cpi->frame_flags |= FRAMEFLAGS_GOLDEN;
  else
    cpi->frame_flags &= ~FRAMEFLAGS_GOLDEN;

  if (cpi->refresh_alt_ref_frame == 1)
    cpi->frame_flags |= FRAMEFLAGS_ALTREF;
  else
    cpi->frame_flags &= ~FRAMEFLAGS_ALTREF;

  cpi->ref_frame_flags = get_ref_frame_flags(cpi);

  cm->last_frame_type = cm->frame_type;

  av1_rc_postencode_update(cpi, *size);

#if 0
  output_frame_level_debug_stats(cpi);
#endif

  if (cm->frame_type == KEY_FRAME) {
    // Tell the caller that the frame was coded as a key frame
    *frame_flags = cpi->frame_flags | FRAMEFLAGS_KEY;
  } else {
    *frame_flags = cpi->frame_flags & ~FRAMEFLAGS_KEY;
  }

  // Clear the one shot update flags for segmentation map and mode/ref loop
  // filter deltas.
  cm->seg.update_map = 0;
  cm->seg.update_data = 0;
  cm->lf.mode_ref_delta_update = 0;

  // keep track of the last coded dimensions
  cm->last_width = cm->width;
  cm->last_height = cm->height;

  // reset to normal state now that we are done.
  if (!cm->show_existing_frame) cm->last_show_frame = cm->show_frame;

  if (cm->show_frame) {
    av1_swap_mi_and_prev_mi(cm);
    // Don't increment frame counters if this was an altref buffer
    // update not a real frame
    ++cm->current_video_frame;
  }
  cm->prev_frame = cm->cur_frame;
}

static void Pass0Encode(AV1_COMP *cpi, size_t *size, uint8_t *dest,
                        unsigned int *frame_flags) {
  if (cpi->oxcf.rc_mode == AOM_CBR) {
    av1_rc_get_one_pass_cbr_params(cpi);
  } else {
    av1_rc_get_one_pass_vbr_params(cpi);
  }
  encode_frame_to_data_rate(cpi, size, dest, frame_flags);
}

static void Pass2Encode(AV1_COMP *cpi, size_t *size, uint8_t *dest,
                        unsigned int *frame_flags) {
  cpi->allow_encode_breakout = ENCODE_BREAKOUT_ENABLED;
  encode_frame_to_data_rate(cpi, size, dest, frame_flags);

  av1_twopass_postencode_update(cpi);
}

static void init_ref_frame_bufs(AV1_COMMON *cm) {
  int i;
  BufferPool *const pool = cm->buffer_pool;
  cm->new_fb_idx = INVALID_IDX;
  for (i = 0; i < REF_FRAMES; ++i) {
    cm->ref_frame_map[i] = INVALID_IDX;
    pool->frame_bufs[i].ref_count = 0;
  }
}

static void check_initial_width(AV1_COMP *cpi,
#if CONFIG_AOM_HIGHBITDEPTH
                                int use_highbitdepth,
#endif
                                int subsampling_x, int subsampling_y) {
  AV1_COMMON *const cm = &cpi->common;

  if (!cpi->initial_width ||
#if CONFIG_AOM_HIGHBITDEPTH
      cm->use_highbitdepth != use_highbitdepth ||
#endif
      cm->subsampling_x != subsampling_x ||
      cm->subsampling_y != subsampling_y) {
    cm->subsampling_x = subsampling_x;
    cm->subsampling_y = subsampling_y;
#if CONFIG_AOM_HIGHBITDEPTH
    cm->use_highbitdepth = use_highbitdepth;
#endif

    alloc_raw_frame_buffers(cpi);
    init_ref_frame_bufs(cm);
    alloc_util_frame_buffers(cpi);

    init_motion_estimation(cpi);  // TODO(agrange) This can be removed.

    cpi->initial_width = cm->width;
    cpi->initial_height = cm->height;
    cpi->initial_mbs = cm->MBs;
  }
}

int av1_receive_raw_frame(AV1_COMP *cpi, unsigned int frame_flags,
                          YV12_BUFFER_CONFIG *sd, int64_t time_stamp,
                          int64_t end_time) {
  AV1_COMMON *cm = &cpi->common;
  struct aom_usec_timer timer;
  int res = 0;
  const int subsampling_x = sd->subsampling_x;
  const int subsampling_y = sd->subsampling_y;
#if CONFIG_AOM_HIGHBITDEPTH
  const int use_highbitdepth = sd->flags & YV12_FLAG_HIGHBITDEPTH;
  check_initial_width(cpi, use_highbitdepth, subsampling_x, subsampling_y);
#else
  check_initial_width(cpi, subsampling_x, subsampling_y);
#endif  // CONFIG_AOM_HIGHBITDEPTH

  aom_usec_timer_start(&timer);

  if (av1_lookahead_push(cpi->lookahead, sd, time_stamp, end_time,
#if CONFIG_AOM_HIGHBITDEPTH
                         use_highbitdepth,
#endif  // CONFIG_AOM_HIGHBITDEPTH
                         frame_flags))
    res = -1;
  aom_usec_timer_mark(&timer);
  cpi->time_receive_data += aom_usec_timer_elapsed(&timer);

  if ((cm->profile == PROFILE_0 || cm->profile == PROFILE_2) &&
      (subsampling_x != 1 || subsampling_y != 1)) {
    aom_internal_error(&cm->error, AOM_CODEC_INVALID_PARAM,
                       "Non-4:2:0 color format requires profile 1 or 3");
    res = -1;
  }
  if ((cm->profile == PROFILE_1 || cm->profile == PROFILE_3) &&
      (subsampling_x == 1 && subsampling_y == 1)) {
    aom_internal_error(&cm->error, AOM_CODEC_INVALID_PARAM,
                       "4:2:0 color format requires profile 0 or 2");
    res = -1;
  }

  return res;
}

static int frame_is_reference(const AV1_COMP *cpi) {
  const AV1_COMMON *cm = &cpi->common;

  return cm->frame_type == KEY_FRAME || cpi->refresh_last_frame ||
         cpi->refresh_golden_frame || cpi->refresh_alt_ref_frame ||
         cm->refresh_frame_context != REFRESH_FRAME_CONTEXT_OFF ||
         cm->lf.mode_ref_delta_update || cm->seg.update_map ||
         cm->seg.update_data;
}

static void adjust_frame_rate(AV1_COMP *cpi,
                              const struct lookahead_entry *source) {
  int64_t this_duration;
  int step = 0;

  if (source->ts_start == cpi->first_time_stamp_ever) {
    this_duration = source->ts_end - source->ts_start;
    step = 1;
  } else {
    int64_t last_duration =
        cpi->last_end_time_stamp_seen - cpi->last_time_stamp_seen;

    this_duration = source->ts_end - cpi->last_end_time_stamp_seen;

    // do a step update if the duration changes by 10%
    if (last_duration)
      step = (int)((this_duration - last_duration) * 10 / last_duration);
  }

  if (this_duration) {
    if (step) {
      av1_new_framerate(cpi, 10000000.0 / this_duration);
    } else {
      // Average this frame's rate into the last second's average
      // frame rate. If we haven't seen 1 second yet, then average
      // over the whole interval seen.
      const double interval = AOMMIN(
          (double)(source->ts_end - cpi->first_time_stamp_ever), 10000000.0);
      double avg_duration = 10000000.0 / cpi->framerate;
      avg_duration *= (interval - avg_duration + this_duration);
      avg_duration /= interval;

      av1_new_framerate(cpi, 10000000.0 / avg_duration);
    }
  }
  cpi->last_time_stamp_seen = source->ts_start;
  cpi->last_end_time_stamp_seen = source->ts_end;
}

// Returns 0 if this is not an alt ref else the offset of the source frame
// used as the arf midpoint.
static int get_arf_src_index(AV1_COMP *cpi) {
  RATE_CONTROL *const rc = &cpi->rc;
  int arf_src_index = 0;
  if (is_altref_enabled(cpi)) {
    if (cpi->oxcf.pass == 2) {
      const GF_GROUP *const gf_group = &cpi->twopass.gf_group;
      if (gf_group->update_type[gf_group->index] == ARF_UPDATE) {
        arf_src_index = gf_group->arf_src_offset[gf_group->index];
      }
    } else if (rc->source_alt_ref_pending) {
      arf_src_index = rc->frames_till_gf_update_due;
    }
  }
  return arf_src_index;
}

static void check_src_altref(AV1_COMP *cpi,
                             const struct lookahead_entry *source) {
  RATE_CONTROL *const rc = &cpi->rc;

  if (cpi->oxcf.pass == 2) {
    const GF_GROUP *const gf_group = &cpi->twopass.gf_group;
    rc->is_src_frame_alt_ref =
        (gf_group->update_type[gf_group->index] == OVERLAY_UPDATE);
  } else {
    rc->is_src_frame_alt_ref =
        cpi->alt_ref_source && (source == cpi->alt_ref_source);
  }

  if (rc->is_src_frame_alt_ref) {
    // Current frame is an ARF overlay frame.
    cpi->alt_ref_source = NULL;

    // Don't refresh the last buffer for an ARF overlay frame. It will
    // become the GF so preserve last as an alternative prediction option.
    cpi->refresh_last_frame = 0;
  }
}

#if CONFIG_INTERNAL_STATS
extern double av1_get_blockiness(const unsigned char *img1, int img1_pitch,
                                 const unsigned char *img2, int img2_pitch,
                                 int width, int height);

static void adjust_image_stat(double y, double u, double v, double all,
                              ImageStat *s) {
  s->stat[Y] += y;
  s->stat[U] += u;
  s->stat[V] += v;
  s->stat[ALL] += all;
  s->worst = AOMMIN(s->worst, all);
}

static void compute_internal_stats(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  double samples = 0.0;
  uint32_t in_bit_depth = 8;
  uint32_t bit_depth = 8;

#if CONFIG_AOM_HIGHBITDEPTH
  if (cm->use_highbitdepth) {
    in_bit_depth = cpi->oxcf.input_bit_depth;
    bit_depth = cm->bit_depth;
  }
#endif
  if (cm->show_frame) {
    const YV12_BUFFER_CONFIG *orig = cpi->Source;
    const YV12_BUFFER_CONFIG *recon = cpi->common.frame_to_show;
    double y, u, v, frame_all;

    cpi->count++;
    if (cpi->b_calculate_psnr) {
      PSNR_STATS psnr;
      double frame_ssim2 = 0.0, weight = 0.0;
      aom_clear_system_state();
// TODO(yaowu): unify these two versions into one.
#if CONFIG_AOM_HIGHBITDEPTH
      aom_calc_highbd_psnr(orig, recon, &psnr, bit_depth, in_bit_depth);
#else
      aom_calc_psnr(orig, recon, &psnr);
#endif  // CONFIG_AOM_HIGHBITDEPTH

      adjust_image_stat(psnr.psnr[1], psnr.psnr[2], psnr.psnr[3], psnr.psnr[0],
                        &cpi->psnr);
      cpi->total_sq_error += psnr.sse[0];
      cpi->total_samples += psnr.samples[0];
      samples = psnr.samples[0];
// TODO(yaowu): unify these two versions into one.
#if CONFIG_AOM_HIGHBITDEPTH
      if (cm->use_highbitdepth)
        frame_ssim2 =
            aom_highbd_calc_ssim(orig, recon, &weight, bit_depth, in_bit_depth);
      else
        frame_ssim2 = aom_calc_ssim(orig, recon, &weight);
#else
      frame_ssim2 = aom_calc_ssim(orig, recon, &weight);
#endif  // CONFIG_AOM_HIGHBITDEPTH

      cpi->worst_ssim = AOMMIN(cpi->worst_ssim, frame_ssim2);
      cpi->summed_quality += frame_ssim2 * weight;
      cpi->summed_weights += weight;

#if 0
      {
        FILE *f = fopen("q_used.stt", "a");
        fprintf(f, "%5d : Y%f7.3:U%f7.3:V%f7.3:F%f7.3:S%7.3f\n",
                cpi->common.current_video_frame, y2, u2, v2,
                frame_psnr2, frame_ssim2);
        fclose(f);
      }
#endif
    }
    if (cpi->b_calculate_blockiness) {
#if CONFIG_AOM_HIGHBITDEPTH
      if (!cm->use_highbitdepth)
#endif
      {
        const double frame_blockiness =
            av1_get_blockiness(orig->y_buffer, orig->y_stride, recon->y_buffer,
                               recon->y_stride, orig->y_width, orig->y_height);
        cpi->worst_blockiness = AOMMAX(cpi->worst_blockiness, frame_blockiness);
        cpi->total_blockiness += frame_blockiness;
      }

      if (cpi->b_calculate_consistency) {
#if CONFIG_AOM_HIGHBITDEPTH
        if (!cm->use_highbitdepth)
#endif
        {
          const double this_inconsistency = aom_get_ssim_metrics(
              orig->y_buffer, orig->y_stride, recon->y_buffer, recon->y_stride,
              orig->y_width, orig->y_height, cpi->ssim_vars, &cpi->metrics, 1);

          const double peak = (double)((1 << in_bit_depth) - 1);
          const double consistency =
              aom_sse_to_psnr(samples, peak, cpi->total_inconsistency);
          if (consistency > 0.0)
            cpi->worst_consistency =
                AOMMIN(cpi->worst_consistency, consistency);
          cpi->total_inconsistency += this_inconsistency;
        }
      }
    }

    frame_all =
        aom_calc_fastssim(orig, recon, &y, &u, &v, bit_depth, in_bit_depth);
    adjust_image_stat(y, u, v, frame_all, &cpi->fastssim);
    frame_all = aom_psnrhvs(orig, recon, &y, &u, &v, bit_depth, in_bit_depth);
    adjust_image_stat(y, u, v, frame_all, &cpi->psnrhvs);
  }
}
#endif  // CONFIG_INTERNAL_STATS

int av1_get_compressed_data(AV1_COMP *cpi, unsigned int *frame_flags,
                            size_t *size, uint8_t *dest, int64_t *time_stamp,
                            int64_t *time_end, int flush) {
  const AV1EncoderConfig *const oxcf = &cpi->oxcf;
  AV1_COMMON *const cm = &cpi->common;
  BufferPool *const pool = cm->buffer_pool;
  RATE_CONTROL *const rc = &cpi->rc;
  struct aom_usec_timer cmptimer;
  YV12_BUFFER_CONFIG *force_src_buffer = NULL;
  struct lookahead_entry *last_source = NULL;
  struct lookahead_entry *source = NULL;
  int arf_src_index;
  int i;

  aom_usec_timer_start(&cmptimer);

  av1_set_high_precision_mv(cpi, ALTREF_HIGH_PRECISION_MV);

  // Is multi-arf enabled.
  // Note that at the moment multi_arf is only configured for 2 pass VBR
  if ((oxcf->pass == 2) && (cpi->oxcf.enable_auto_arf > 1))
    cpi->multi_arf_allowed = 1;
  else
    cpi->multi_arf_allowed = 0;

  // Normal defaults
  cm->reset_frame_context = RESET_FRAME_CONTEXT_NONE;
  cm->refresh_frame_context = oxcf->error_resilient_mode
                                  ? REFRESH_FRAME_CONTEXT_OFF
                                  : oxcf->frame_parallel_decoding_mode
                                        ? REFRESH_FRAME_CONTEXT_FORWARD
                                        : REFRESH_FRAME_CONTEXT_BACKWARD;

  cpi->refresh_last_frame = 1;
  cpi->refresh_golden_frame = 0;
  cpi->refresh_alt_ref_frame = 0;

  // Should we encode an arf frame.
  arf_src_index = get_arf_src_index(cpi);

  if (arf_src_index) {
    assert(arf_src_index <= rc->frames_to_key);

    if ((source = av1_lookahead_peek(cpi->lookahead, arf_src_index)) != NULL) {
      cpi->alt_ref_source = source;

      if (oxcf->arnr_max_frames > 0) {
        // Produce the filtered ARF frame.
        av1_temporal_filter(cpi, arf_src_index);
        aom_extend_frame_borders(&cpi->alt_ref_buffer);
        force_src_buffer = &cpi->alt_ref_buffer;
      }

      cm->show_frame = 0;
      cm->intra_only = 0;
      cpi->refresh_alt_ref_frame = 1;
      cpi->refresh_golden_frame = 0;
      cpi->refresh_last_frame = 0;
      rc->is_src_frame_alt_ref = 0;
      rc->source_alt_ref_pending = 0;
    } else {
      rc->source_alt_ref_pending = 0;
    }
  }

  if (!source) {
    // Get last frame source.
    if (cm->current_video_frame > 0) {
      if ((last_source = av1_lookahead_peek(cpi->lookahead, -1)) == NULL)
        return -1;
    }

    // Read in the source frame.
    source = av1_lookahead_pop(cpi->lookahead, flush);

    if (source != NULL) {
      cm->show_frame = 1;
      cm->intra_only = 0;

      // Check to see if the frame should be encoded as an arf overlay.
      check_src_altref(cpi, source);
    }
  }

  if (source) {
    cpi->un_scaled_source = cpi->Source =
        force_src_buffer ? force_src_buffer : &source->img;

    cpi->unscaled_last_source = last_source != NULL ? &last_source->img : NULL;

    *time_stamp = source->ts_start;
    *time_end = source->ts_end;
    *frame_flags = (source->flags & AOM_EFLAG_FORCE_KF) ? FRAMEFLAGS_KEY : 0;

  } else {
    *size = 0;
    if (flush && oxcf->pass == 1 && !cpi->twopass.first_pass_done) {
      av1_end_first_pass(cpi); /* get last stats packet */
      cpi->twopass.first_pass_done = 1;
    }
    return -1;
  }

  if (source->ts_start < cpi->first_time_stamp_ever) {
    cpi->first_time_stamp_ever = source->ts_start;
    cpi->last_end_time_stamp_seen = source->ts_start;
  }

  // Clear down mmx registers
  aom_clear_system_state();

  // adjust frame rates based on timestamps given
  if (cm->show_frame) {
    adjust_frame_rate(cpi, source);
  }

  // Find a free buffer for the new frame, releasing the reference previously
  // held.
  if (cm->new_fb_idx != INVALID_IDX) {
    --pool->frame_bufs[cm->new_fb_idx].ref_count;
  }
  cm->new_fb_idx = get_free_fb(cm);

  if (cm->new_fb_idx == INVALID_IDX) return -1;

  cm->cur_frame = &pool->frame_bufs[cm->new_fb_idx];

  if (cpi->multi_arf_allowed) {
    if (cm->frame_type == KEY_FRAME) {
      init_buffer_indices(cpi);
    } else if (oxcf->pass == 2) {
      const GF_GROUP *const gf_group = &cpi->twopass.gf_group;
      cpi->alt_fb_idx = gf_group->arf_ref_idx[gf_group->index];
    }
  }

  // Start with a 0 size frame.
  *size = 0;

  cpi->frame_flags = *frame_flags;

  if (oxcf->pass == 2) {
    av1_rc_get_second_pass_params(cpi);
  } else if (oxcf->pass == 1) {
    set_frame_size(cpi);
  }

  if (cpi->oxcf.pass != 0 || frame_is_intra_only(cm) == 1) {
    for (i = 0; i < MAX_REF_FRAMES; ++i) cpi->scaled_ref_idx[i] = INVALID_IDX;
  }

#if CONFIG_AOM_QM
  cm->using_qmatrix = cpi->oxcf.using_qm;
  cm->min_qmlevel = cpi->oxcf.qm_minlevel;
  cm->max_qmlevel = cpi->oxcf.qm_maxlevel;
#endif

  if (oxcf->pass == 1) {
    cpi->td.mb.e_mbd.lossless[0] = is_lossless_requested(oxcf);
    av1_first_pass(cpi, source);
  } else if (oxcf->pass == 2) {
    Pass2Encode(cpi, size, dest, frame_flags);
  } else {
    // One pass encode
    Pass0Encode(cpi, size, dest, frame_flags);
  }

  if (cm->refresh_frame_context != REFRESH_FRAME_CONTEXT_OFF)
    cm->frame_contexts[cm->frame_context_idx] = *cm->fc;

  // No frame encoded, or frame was dropped, release scaled references.
  if ((*size == 0) && (frame_is_intra_only(cm) == 0)) {
    release_scaled_references(cpi);
  }

  if (*size > 0) {
    cpi->droppable = !frame_is_reference(cpi);
  }

  aom_usec_timer_mark(&cmptimer);
  cpi->time_compress_data += aom_usec_timer_elapsed(&cmptimer);

  if (cpi->b_calculate_psnr && oxcf->pass != 1 && cm->show_frame)
    generate_psnr_packet(cpi);

#if CONFIG_INTERNAL_STATS
  if (oxcf->pass != 1) {
    compute_internal_stats(cpi);
    cpi->bytes += (int)(*size);
  }
#endif

  aom_clear_system_state();
  return 0;
}

int av1_get_preview_raw_frame(AV1_COMP *cpi, YV12_BUFFER_CONFIG *dest) {
  AV1_COMMON *cm = &cpi->common;

  if (!cm->show_frame) {
    return -1;
  } else {
    int ret;
    if (cm->frame_to_show) {
      *dest = *cm->frame_to_show;
      dest->y_width = cm->width;
      dest->y_height = cm->height;
      dest->uv_width = cm->width >> cm->subsampling_x;
      dest->uv_height = cm->height >> cm->subsampling_y;
      ret = 0;
    } else {
      ret = -1;
    }
    aom_clear_system_state();
    return ret;
  }
}

int av1_set_internal_size(AV1_COMP *cpi, AOM_SCALING horiz_mode,
                          AOM_SCALING vert_mode) {
  AV1_COMMON *cm = &cpi->common;
  int hr = 0, hs = 0, vr = 0, vs = 0;

  if (horiz_mode > ONETWO || vert_mode > ONETWO) return -1;

  Scale2Ratio(horiz_mode, &hr, &hs);
  Scale2Ratio(vert_mode, &vr, &vs);

  // always go to the next whole number
  cm->width = (hs - 1 + cpi->oxcf.width * hr) / hs;
  cm->height = (vs - 1 + cpi->oxcf.height * vr) / vs;
  assert(cm->width <= cpi->initial_width);
  assert(cm->height <= cpi->initial_height);

  update_frame_size(cpi);

  return 0;
}

int av1_set_size_literal(AV1_COMP *cpi, unsigned int width,
                         unsigned int height) {
  AV1_COMMON *cm = &cpi->common;
#if CONFIG_AOM_HIGHBITDEPTH
  check_initial_width(cpi, cm->use_highbitdepth, 1, 1);
#else
  check_initial_width(cpi, 1, 1);
#endif  // CONFIG_AOM_HIGHBITDEPTH

  if (width) {
    cm->width = width;
    if (cm->width > cpi->initial_width) {
      cm->width = cpi->initial_width;
      printf("Warning: Desired width too large, changed to %d\n", cm->width);
    }
  }

  if (height) {
    cm->height = height;
    if (cm->height > cpi->initial_height) {
      cm->height = cpi->initial_height;
      printf("Warning: Desired height too large, changed to %d\n", cm->height);
    }
  }
  assert(cm->width <= cpi->initial_width);
  assert(cm->height <= cpi->initial_height);

  update_frame_size(cpi);

  return 0;
}

int av1_get_quantizer(AV1_COMP *cpi) { return cpi->common.base_qindex; }

void av1_apply_encoding_flags(AV1_COMP *cpi, aom_enc_frame_flags_t flags) {
  if (flags &
      (AOM_EFLAG_NO_REF_LAST | AOM_EFLAG_NO_REF_GF | AOM_EFLAG_NO_REF_ARF)) {
    int ref = 7;

    if (flags & AOM_EFLAG_NO_REF_LAST) ref ^= AOM_LAST_FLAG;

    if (flags & AOM_EFLAG_NO_REF_GF) ref ^= AOM_GOLD_FLAG;

    if (flags & AOM_EFLAG_NO_REF_ARF) ref ^= AOM_ALT_FLAG;

    av1_use_as_reference(cpi, ref);
  }

  if (flags &
      (AOM_EFLAG_NO_UPD_LAST | AOM_EFLAG_NO_UPD_GF | AOM_EFLAG_NO_UPD_ARF |
       AOM_EFLAG_FORCE_GF | AOM_EFLAG_FORCE_ARF)) {
    int upd = 7;

    if (flags & AOM_EFLAG_NO_UPD_LAST) upd ^= AOM_LAST_FLAG;

    if (flags & AOM_EFLAG_NO_UPD_GF) upd ^= AOM_GOLD_FLAG;

    if (flags & AOM_EFLAG_NO_UPD_ARF) upd ^= AOM_ALT_FLAG;

    av1_update_reference(cpi, upd);
  }

  if (flags & AOM_EFLAG_NO_UPD_ENTROPY) {
    av1_update_entropy(cpi, 0);
  }
}
