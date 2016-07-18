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

#include "aom_mem/aom_mem.h"
#include "aom_ports/mem.h"

#include "av1/common/blockd.h"
#include "av1/common/common.h"
#include "av1/common/entropy.h"
#if CONFIG_COEFFICIENT_RANGE_CHECKING
#include "av1/common/idct.h"
#endif

#include "av1/decoder/detokenize.h"

#define EOB_CONTEXT_NODE 0
#define ZERO_CONTEXT_NODE 1
#define ONE_CONTEXT_NODE 2
#define LOW_VAL_CONTEXT_NODE 0
#define TWO_CONTEXT_NODE 1
#define THREE_CONTEXT_NODE 2
#define HIGH_LOW_CONTEXT_NODE 3
#define CAT_ONE_CONTEXT_NODE 4
#define CAT_THREEFOUR_CONTEXT_NODE 5
#define CAT_THREE_CONTEXT_NODE 6
#define CAT_FIVE_CONTEXT_NODE 7

#define INCREMENT_COUNT(token)                   \
  do {                                           \
    if (counts) ++coef_counts[band][ctx][token]; \
  } while (0)

static INLINE int read_coeff(const aom_prob *probs, int n, aom_reader *r) {
  int i, val = 0;
  for (i = 0; i < n; ++i) val = (val << 1) | aom_read(r, probs[i]);
  return val;
}

#if CONFIG_AOM_QM
static int decode_coefs(const MACROBLOCKD *xd, PLANE_TYPE type,
                        tran_low_t *dqcoeff, TX_SIZE tx_size, const int16_t *dq,
                        int ctx, const int16_t *scan, const int16_t *nb,
                        aom_reader *r, const qm_val_t *iqm[2][TX_SIZES])
#else
static int decode_coefs(const MACROBLOCKD *xd, PLANE_TYPE type,
                        tran_low_t *dqcoeff, TX_SIZE tx_size, const int16_t *dq,
                        int ctx, const int16_t *scan, const int16_t *nb,
                        aom_reader *r)
#endif
{
  FRAME_COUNTS *counts = xd->counts;
  const int max_eob = 16 << (tx_size << 1);
  const FRAME_CONTEXT *const fc = xd->fc;
  const int ref = is_inter_block(&xd->mi[0]->mbmi);
#if CONFIG_AOM_QM
  const qm_val_t *iqmatrix = iqm[!ref][tx_size];
#endif
  int band, c = 0;
  const aom_prob(*coef_probs)[COEFF_CONTEXTS][UNCONSTRAINED_NODES] =
      fc->coef_probs[tx_size][type][ref];
  const aom_prob *prob;
  unsigned int(*coef_counts)[COEFF_CONTEXTS][UNCONSTRAINED_NODES + 1];
  unsigned int(*eob_branch_count)[COEFF_CONTEXTS];
  uint8_t token_cache[32 * 32];
  const uint8_t *band_translate = get_band_translate(tx_size);
  const int dq_shift = (tx_size == TX_32X32);
  int v, token;
  int16_t dqv = dq[0];
  const uint8_t *cat1_prob;
  const uint8_t *cat2_prob;
  const uint8_t *cat3_prob;
  const uint8_t *cat4_prob;
  const uint8_t *cat5_prob;
  const uint8_t *cat6_prob;

  if (counts) {
    coef_counts = counts->coef[tx_size][type][ref];
    eob_branch_count = counts->eob_branch[tx_size][type][ref];
  }

#if CONFIG_AOM_HIGHBITDEPTH
  if (xd->bd > AOM_BITS_8) {
    if (xd->bd == AOM_BITS_10) {
      cat1_prob = av1_cat1_prob_high10;
      cat2_prob = av1_cat2_prob_high10;
      cat3_prob = av1_cat3_prob_high10;
      cat4_prob = av1_cat4_prob_high10;
      cat5_prob = av1_cat5_prob_high10;
      cat6_prob = av1_cat6_prob_high10;
    } else {
      cat1_prob = av1_cat1_prob_high12;
      cat2_prob = av1_cat2_prob_high12;
      cat3_prob = av1_cat3_prob_high12;
      cat4_prob = av1_cat4_prob_high12;
      cat5_prob = av1_cat5_prob_high12;
      cat6_prob = av1_cat6_prob_high12;
    }
  } else {
    cat1_prob = av1_cat1_prob;
    cat2_prob = av1_cat2_prob;
    cat3_prob = av1_cat3_prob;
    cat4_prob = av1_cat4_prob;
    cat5_prob = av1_cat5_prob;
    cat6_prob = av1_cat6_prob;
  }
#else
  cat1_prob = av1_cat1_prob;
  cat2_prob = av1_cat2_prob;
  cat3_prob = av1_cat3_prob;
  cat4_prob = av1_cat4_prob;
  cat5_prob = av1_cat5_prob;
  cat6_prob = av1_cat6_prob;
#endif

  while (c < max_eob) {
    int val = -1;
    band = *band_translate++;
    prob = coef_probs[band][ctx];
    if (counts) ++eob_branch_count[band][ctx];
    if (!aom_read(r, prob[EOB_CONTEXT_NODE])) {
      INCREMENT_COUNT(EOB_MODEL_TOKEN);
      break;
    }

    while (!aom_read(r, prob[ZERO_CONTEXT_NODE])) {
      INCREMENT_COUNT(ZERO_TOKEN);
      dqv = dq[1];
      token_cache[scan[c]] = 0;
      ++c;
      if (c >= max_eob) return c;  // zero tokens at the end (no eob token)
      ctx = get_coef_context(nb, token_cache, c);
      band = *band_translate++;
      prob = coef_probs[band][ctx];
    }

    if (!aom_read(r, prob[ONE_CONTEXT_NODE])) {
      INCREMENT_COUNT(ONE_TOKEN);
      token = ONE_TOKEN;
      val = 1;
    } else {
      INCREMENT_COUNT(TWO_TOKEN);
      token = aom_read_tree(r, av1_coef_con_tree,
                            av1_pareto8_full[prob[PIVOT_NODE] - 1]);
      switch (token) {
        case TWO_TOKEN:
        case THREE_TOKEN:
        case FOUR_TOKEN: val = token; break;
        case CATEGORY1_TOKEN:
          val = CAT1_MIN_VAL + read_coeff(cat1_prob, 1, r);
          break;
        case CATEGORY2_TOKEN:
          val = CAT2_MIN_VAL + read_coeff(cat2_prob, 2, r);
          break;
        case CATEGORY3_TOKEN:
          val = CAT3_MIN_VAL + read_coeff(cat3_prob, 3, r);
          break;
        case CATEGORY4_TOKEN:
          val = CAT4_MIN_VAL + read_coeff(cat4_prob, 4, r);
          break;
        case CATEGORY5_TOKEN:
          val = CAT5_MIN_VAL + read_coeff(cat5_prob, 5, r);
          break;
        case CATEGORY6_TOKEN: {
#if CONFIG_MISC_FIXES
          const int skip_bits = TX_SIZES - 1 - tx_size;
#else
          const int skip_bits = 0;
#endif
          const uint8_t *cat6p = cat6_prob + skip_bits;
#if CONFIG_AOM_HIGHBITDEPTH
          switch (xd->bd) {
            case AOM_BITS_8:
              val = CAT6_MIN_VAL + read_coeff(cat6p, 14 - skip_bits, r);
              break;
            case AOM_BITS_10:
              val = CAT6_MIN_VAL + read_coeff(cat6p, 16 - skip_bits, r);
              break;
            case AOM_BITS_12:
              val = CAT6_MIN_VAL + read_coeff(cat6p, 18 - skip_bits, r);
              break;
            default: assert(0); return -1;
          }
#else
          val = CAT6_MIN_VAL + read_coeff(cat6p, 14 - skip_bits, r);
#endif
          break;
        }
      }
    }
#if CONFIG_AOM_QM
    dqv = ((iqmatrix[scan[c]] * (int)dqv) + (1 << (AOM_QM_BITS - 1))) >>
          AOM_QM_BITS;
#endif
    v = (val * dqv) >> dq_shift;
#if CONFIG_COEFFICIENT_RANGE_CHECKING
#if CONFIG_AOM_HIGHBITDEPTH
    dqcoeff[scan[c]] = highbd_check_range((aom_read_bit(r) ? -v : v), xd->bd);
#else
    dqcoeff[scan[c]] = check_range(aom_read_bit(r) ? -v : v);
#endif  // CONFIG_AOM_HIGHBITDEPTH
#else
    dqcoeff[scan[c]] = aom_read_bit(r) ? -v : v;
#endif  // CONFIG_COEFFICIENT_RANGE_CHECKING
    token_cache[scan[c]] = av1_pt_energy_class[token];
    ++c;
    ctx = get_coef_context(nb, token_cache, c);
    dqv = dq[1];
  }

  return c;
}

int av1_decode_block_tokens(MACROBLOCKD *xd, int plane, const scan_order *sc,
                            int x, int y, TX_SIZE tx_size, aom_reader *r,
                            int seg_id) {
  struct macroblockd_plane *const pd = &xd->plane[plane];
  const int16_t *const dequant = pd->seg_dequant[seg_id];
  const int ctx =
      get_entropy_context(tx_size, pd->above_context + x, pd->left_context + y);
#if CONFIG_AOM_QM
  const int eob =
      decode_coefs(xd, pd->plane_type, pd->dqcoeff, tx_size, dequant, ctx,
                   sc->scan, sc->neighbors, r, pd->seg_iqmatrix[seg_id]);
#else
  const int eob = decode_coefs(xd, pd->plane_type, pd->dqcoeff, tx_size,
                               dequant, ctx, sc->scan, sc->neighbors, r);
#endif
  av1_set_contexts(xd, pd, tx_size, eob > 0, x, y);
  return eob;
}
