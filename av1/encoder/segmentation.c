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

#include "aom_mem/aom_mem.h"
#include "aom_ports/system_state.h"

#include "av1/common/pred_common.h"
#include "av1/common/tile_common.h"

#include "av1/encoder/cost.h"
#include "av1/encoder/segmentation.h"
#include "av1/encoder/subexp.h"

void av1_enable_segmentation(struct segmentation *seg) {
  seg->enabled = 1;
  seg->update_map = 1;
  seg->update_data = 1;
}

void av1_disable_segmentation(struct segmentation *seg) {
  seg->enabled = 0;
  seg->update_map = 0;
  seg->update_data = 0;
}

void av1_set_segment_data(struct segmentation *seg, signed char *feature_data,
                          unsigned char abs_delta) {
  seg->abs_delta = abs_delta;

  memcpy(seg->feature_data, feature_data, sizeof(seg->feature_data));
}
void av1_disable_segfeature(struct segmentation *seg, int segment_id,
                            SEG_LVL_FEATURES feature_id) {
  seg->feature_mask[segment_id] &= ~(1 << feature_id);
}

void av1_clear_segdata(struct segmentation *seg, int segment_id,
                       SEG_LVL_FEATURES feature_id) {
  seg->feature_data[segment_id][feature_id] = 0;
}

// Based on set of segment counts calculate a probability tree
static void calc_segtree_probs(unsigned *segcounts,
                               aom_prob *segment_tree_probs,
                               const aom_prob *cur_tree_probs) {
  // Work out probabilities of each segment
  const unsigned cc[4] = { segcounts[0] + segcounts[1],
                           segcounts[2] + segcounts[3],
                           segcounts[4] + segcounts[5],
                           segcounts[6] + segcounts[7] };
  const unsigned ccc[2] = { cc[0] + cc[1], cc[2] + cc[3] };
#if CONFIG_MISC_FIXES
  int i;
#endif

  segment_tree_probs[0] = get_binary_prob(ccc[0], ccc[1]);
  segment_tree_probs[1] = get_binary_prob(cc[0], cc[1]);
  segment_tree_probs[2] = get_binary_prob(cc[2], cc[3]);
  segment_tree_probs[3] = get_binary_prob(segcounts[0], segcounts[1]);
  segment_tree_probs[4] = get_binary_prob(segcounts[2], segcounts[3]);
  segment_tree_probs[5] = get_binary_prob(segcounts[4], segcounts[5]);
  segment_tree_probs[6] = get_binary_prob(segcounts[6], segcounts[7]);

#if CONFIG_MISC_FIXES
  for (i = 0; i < 7; i++) {
    const unsigned *ct =
        i == 0 ? ccc : i < 3 ? cc + (i & 2) : segcounts + (i - 3) * 2;
    av1_prob_diff_update_savings_search(
        ct, cur_tree_probs[i], &segment_tree_probs[i], DIFF_UPDATE_PROB);
  }
#else
  (void)cur_tree_probs;
#endif
}

// Based on set of segment counts and probabilities calculate a cost estimate
static int cost_segmap(unsigned *segcounts, aom_prob *probs) {
  const int c01 = segcounts[0] + segcounts[1];
  const int c23 = segcounts[2] + segcounts[3];
  const int c45 = segcounts[4] + segcounts[5];
  const int c67 = segcounts[6] + segcounts[7];
  const int c0123 = c01 + c23;
  const int c4567 = c45 + c67;

  // Cost the top node of the tree
  int cost = c0123 * av1_cost_zero(probs[0]) + c4567 * av1_cost_one(probs[0]);

  // Cost subsequent levels
  if (c0123 > 0) {
    cost += c01 * av1_cost_zero(probs[1]) + c23 * av1_cost_one(probs[1]);

    if (c01 > 0)
      cost += segcounts[0] * av1_cost_zero(probs[3]) +
              segcounts[1] * av1_cost_one(probs[3]);
    if (c23 > 0)
      cost += segcounts[2] * av1_cost_zero(probs[4]) +
              segcounts[3] * av1_cost_one(probs[4]);
  }

  if (c4567 > 0) {
    cost += c45 * av1_cost_zero(probs[2]) + c67 * av1_cost_one(probs[2]);

    if (c45 > 0)
      cost += segcounts[4] * av1_cost_zero(probs[5]) +
              segcounts[5] * av1_cost_one(probs[5]);
    if (c67 > 0)
      cost += segcounts[6] * av1_cost_zero(probs[6]) +
              segcounts[7] * av1_cost_one(probs[6]);
  }

  return cost;
}

static void calc_segmap_costs(AV1_COMMON *cm, 
                              unsigned *no_pred_segcounts,
                              unsigned (*temporal_predictor_count)[2], unsigned *t_unpred_seg_counts,
                              int *no_pred,
                              int *t_pred)
{
  int i;
  int no_pred_cost;
  int t_pred_cost = INT_MAX;

  aom_prob no_pred_tree[SEG_TREE_PROBS];
  aom_prob t_pred_tree[SEG_TREE_PROBS];
  aom_prob t_nopred_prob[PREDICTION_PROBS];
  struct segmentation_probs *segp = &cm->fc->seg;

  // Work out probability tree for coding segments without prediction
  // and the cost.
  calc_segtree_probs(no_pred_segcounts, no_pred_tree, segp->tree_probs);
  no_pred_cost = cost_segmap(no_pred_segcounts, no_pred_tree);

  // Key frames cannot use temporal prediction
  if (!frame_is_intra_only(cm) && !cm->error_resilient_mode) {
    // Work out probability tree for coding those segments not
    // predicted using the temporal method and the cost.
    calc_segtree_probs(t_unpred_seg_counts, t_pred_tree, segp->tree_probs);
    t_pred_cost = cost_segmap(t_unpred_seg_counts, t_pred_tree);

    // Add in the cost of the signaling for each prediction context.
    for (i = 0; i < PREDICTION_PROBS; i++) {
      const int count0 = temporal_predictor_count[i][0];
      const int count1 = temporal_predictor_count[i][1];

      t_nopred_prob[i] = get_binary_prob(count0, count1);
      av1_prob_diff_update_savings_search(temporal_predictor_count[i],
                                          segp->pred_probs[i],
                                          &t_nopred_prob[i], DIFF_UPDATE_PROB);

      // Add in the predictor signaling cost
      t_pred_cost += count0 * av1_cost_zero(t_nopred_prob[i]) +
                     count1 * av1_cost_one(t_nopred_prob[i]);
    }
  }

  *no_pred = no_pred_cost;
  *t_pred = t_pred_cost;
}

static void update_sb_segcounts(AV1_COMMON *cm, const MACROBLOCKD *xd, int segment_id, BLOCK_SIZE bs, int mi_row, int mi_col,
                                unsigned *no_pred_segcounts,
                                unsigned (*temporal_predictor_count)[2], unsigned *t_unpred_seg_counts) {
  // Count the number of hits on each segment with no prediction
  no_pred_segcounts[segment_id]++;

  // Temporal prediction not allowed on key frames
  if (cm->frame_type != KEY_FRAME) {
    // Test to see if the segment id matches the predicted value.
    const int pred_segment_id =
        get_segment_id(cm, cm->last_frame_seg_map, bs, mi_row, mi_col);
    const int pred_flag = pred_segment_id == segment_id;
    const int pred_context = av1_get_pred_context_seg_id(xd);

    // Store the prediction status for this mb and update counts
    // as appropriate
    xd->mi[0]->mbmi.seg_id_predicted = pred_flag;
    temporal_predictor_count[pred_context][pred_flag]++;

    // Update the "unpredicted" segment count
    if (!pred_flag) t_unpred_seg_counts[segment_id]++;
  }
}

int av1_calc_segmap_cost(AV1_COMMON *cm, MACROBLOCKD *xd, int segment_id, BLOCK_SIZE bs, int mi_row, int mi_col, struct seg_counts *counts) {
  int no_pred_cost;
  int t_pred_cost;

  unsigned(*temporal_predictor_count)[2] = counts->pred;
  unsigned *no_pred_segcounts = counts->tree_total;
  unsigned *t_unpred_seg_counts = counts->tree_mispred;

  update_sb_segcounts(cm, xd, segment_id, bs, mi_row, mi_col, no_pred_segcounts, temporal_predictor_count, t_unpred_seg_counts);

  calc_segmap_costs(cm, no_pred_segcounts, temporal_predictor_count, t_unpred_seg_counts, &no_pred_cost, &t_pred_cost);

  if (t_pred_cost < no_pred_cost) {
    assert(!cm->error_resilient_mode);
    return t_pred_cost;
  } else {
    return no_pred_cost;
  }
}

void av1_choose_segmap_coding_method(AV1_COMMON *cm, MACROBLOCKD *xd) {
  struct segmentation *seg = &cm->seg;

  int no_pred_cost;
  int t_pred_cost;

  unsigned(*temporal_predictor_count)[2] = cm->counts.seg.pred;
  unsigned *no_pred_segcounts = cm->counts.seg.tree_total;
  unsigned *t_unpred_seg_counts = cm->counts.seg.tree_mispred;

  calc_segmap_costs(cm, no_pred_segcounts, temporal_predictor_count, t_unpred_seg_counts, &no_pred_cost, &t_pred_cost);

  // Now choose which coding method to use.
  if (t_pred_cost < no_pred_cost) {
    assert(!cm->error_resilient_mode);
    seg->temporal_update = 1;
#if !CONFIG_MISC_FIXES
    memcpy(segp->tree_probs, t_pred_tree, sizeof(t_pred_tree));
    memcpy(segp->pred_probs, t_nopred_prob, sizeof(t_nopred_prob));
#endif
  } else {
    seg->temporal_update = 0;
#if !CONFIG_MISC_FIXES
    memcpy(segp->tree_probs, no_pred_tree, sizeof(no_pred_tree));
#endif
  }
}

void av1_reset_segment_features(AV1_COMMON *cm) {
  struct segmentation *seg = &cm->seg;
#if !CONFIG_MISC_FIXES
  struct segmentation_probs *segp = &cm->segp;
#endif

  // Set up default state for MB feature flags
  seg->enabled = 0;
  seg->update_map = 0;
  seg->update_data = 0;
#if !CONFIG_MISC_FIXES
  memset(segp->tree_probs, 255, sizeof(segp->tree_probs));
#endif
  av1_clearall_segfeatures(seg);
}

int av1_calc_segment_rdmult(AV1_COMP *const cpi, MACROBLOCK *const x,
                            int8_t segment_id) {
  int segment_qindex;
  AV1_COMMON *const cm = &cpi->common;
  av1_init_plane_quantizers(cpi, x);
  aom_clear_system_state();
  segment_qindex = av1_get_qindex(&cm->seg, segment_id, cm->base_qindex);
  return av1_compute_rd_mult(cpi, segment_qindex + cm->y_dc_delta_q);
}

