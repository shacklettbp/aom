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

// AV1 Set Reference Frame
// ============================
//
// This is an example demonstrating how to overwrite the AV1 encoder's
// internal reference frame. In the sample we set the last frame to the
// current frame. This technique could be used to bounce between two cameras.
//
// The decoder would also have to set the reference frame to the same value
// on the same frame, or the video will become corrupt. The 'test_decode'
// variable is set to 1 in this example that tests if the encoder and decoder
// results are matching.
//
// Usage
// -----
// This example encodes a raw video. And the last argument passed in specifies
// the frame number to update the reference frame on. For example, run
// examples/aom_cx_set_ref vp10 352 288 in.yuv out.ivf 4 30
// The parameter is parsed as follows:
//
//
// Extra Variables
// ---------------
// This example maintains the frame number passed on the command line
// in the `update_frame_num` variable.
//
//
// Configuration
// -------------
//
// The reference frame is updated on the frame specified on the command
// line.
//
// Observing The Effects
// ---------------------
// The encoder and decoder results should be matching when the same reference
// frame setting operation is done in both encoder and decoder. Otherwise,
// the encoder/decoder mismatch would be seen.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aom/aomcx.h"
#include "aom/aom_decoder.h"
#include "aom/aom_encoder.h"

#include "./tools_common.h"
#include "./video_writer.h"

static const char *exec_name;

void usage_exit() {
  fprintf(stderr,
          "Usage: %s <codec> <width> <height> <infile> <outfile> "
          "<frame> <limit(optional)>\n",
          exec_name);
  exit(EXIT_FAILURE);
}

static int compare_img(const aom_image_t *const img1,
                       const aom_image_t *const img2) {
  uint32_t l_w = img1->d_w;
  uint32_t c_w = (img1->d_w + img1->x_chroma_shift) >> img1->x_chroma_shift;
  const uint32_t c_h =
      (img1->d_h + img1->y_chroma_shift) >> img1->y_chroma_shift;
  uint32_t i;
  int match = 1;

  match &= (img1->fmt == img2->fmt);
  match &= (img1->d_w == img2->d_w);
  match &= (img1->d_h == img2->d_h);

  for (i = 0; i < img1->d_h; ++i)
    match &= (memcmp(img1->planes[AOM_PLANE_Y] + i * img1->stride[AOM_PLANE_Y],
                     img2->planes[AOM_PLANE_Y] + i * img2->stride[AOM_PLANE_Y],
                     l_w) == 0);

  for (i = 0; i < c_h; ++i)
    match &= (memcmp(img1->planes[AOM_PLANE_U] + i * img1->stride[AOM_PLANE_U],
                     img2->planes[AOM_PLANE_U] + i * img2->stride[AOM_PLANE_U],
                     c_w) == 0);

  for (i = 0; i < c_h; ++i)
    match &= (memcmp(img1->planes[AOM_PLANE_V] + i * img1->stride[AOM_PLANE_V],
                     img2->planes[AOM_PLANE_V] + i * img2->stride[AOM_PLANE_V],
                     c_w) == 0);

  return match;
}

#define mmin(a, b) ((a) < (b) ? (a) : (b))
static void find_mismatch(const aom_image_t *const img1,
                          const aom_image_t *const img2, int yloc[4],
                          int uloc[4], int vloc[4]) {
  const uint32_t bsize = 64;
  const uint32_t bsizey = bsize >> img1->y_chroma_shift;
  const uint32_t bsizex = bsize >> img1->x_chroma_shift;
  const uint32_t c_w =
      (img1->d_w + img1->x_chroma_shift) >> img1->x_chroma_shift;
  const uint32_t c_h =
      (img1->d_h + img1->y_chroma_shift) >> img1->y_chroma_shift;
  int match = 1;
  uint32_t i, j;
  yloc[0] = yloc[1] = yloc[2] = yloc[3] = -1;
  for (i = 0, match = 1; match && i < img1->d_h; i += bsize) {
    for (j = 0; match && j < img1->d_w; j += bsize) {
      int k, l;
      const int si = mmin(i + bsize, img1->d_h) - i;
      const int sj = mmin(j + bsize, img1->d_w) - j;
      for (k = 0; match && k < si; ++k) {
        for (l = 0; match && l < sj; ++l) {
          if (*(img1->planes[AOM_PLANE_Y] +
                (i + k) * img1->stride[AOM_PLANE_Y] + j + l) !=
              *(img2->planes[AOM_PLANE_Y] +
                (i + k) * img2->stride[AOM_PLANE_Y] + j + l)) {
            yloc[0] = i + k;
            yloc[1] = j + l;
            yloc[2] = *(img1->planes[AOM_PLANE_Y] +
                        (i + k) * img1->stride[AOM_PLANE_Y] + j + l);
            yloc[3] = *(img2->planes[AOM_PLANE_Y] +
                        (i + k) * img2->stride[AOM_PLANE_Y] + j + l);
            match = 0;
            break;
          }
        }
      }
    }
  }

  uloc[0] = uloc[1] = uloc[2] = uloc[3] = -1;
  for (i = 0, match = 1; match && i < c_h; i += bsizey) {
    for (j = 0; match && j < c_w; j += bsizex) {
      int k, l;
      const int si = mmin(i + bsizey, c_h - i);
      const int sj = mmin(j + bsizex, c_w - j);
      for (k = 0; match && k < si; ++k) {
        for (l = 0; match && l < sj; ++l) {
          if (*(img1->planes[AOM_PLANE_U] +
                (i + k) * img1->stride[AOM_PLANE_U] + j + l) !=
              *(img2->planes[AOM_PLANE_U] +
                (i + k) * img2->stride[AOM_PLANE_U] + j + l)) {
            uloc[0] = i + k;
            uloc[1] = j + l;
            uloc[2] = *(img1->planes[AOM_PLANE_U] +
                        (i + k) * img1->stride[AOM_PLANE_U] + j + l);
            uloc[3] = *(img2->planes[AOM_PLANE_U] +
                        (i + k) * img2->stride[AOM_PLANE_U] + j + l);
            match = 0;
            break;
          }
        }
      }
    }
  }
  vloc[0] = vloc[1] = vloc[2] = vloc[3] = -1;
  for (i = 0, match = 1; match && i < c_h; i += bsizey) {
    for (j = 0; match && j < c_w; j += bsizex) {
      int k, l;
      const int si = mmin(i + bsizey, c_h - i);
      const int sj = mmin(j + bsizex, c_w - j);
      for (k = 0; match && k < si; ++k) {
        for (l = 0; match && l < sj; ++l) {
          if (*(img1->planes[AOM_PLANE_V] +
                (i + k) * img1->stride[AOM_PLANE_V] + j + l) !=
              *(img2->planes[AOM_PLANE_V] +
                (i + k) * img2->stride[AOM_PLANE_V] + j + l)) {
            vloc[0] = i + k;
            vloc[1] = j + l;
            vloc[2] = *(img1->planes[AOM_PLANE_V] +
                        (i + k) * img1->stride[AOM_PLANE_V] + j + l);
            vloc[3] = *(img2->planes[AOM_PLANE_V] +
                        (i + k) * img2->stride[AOM_PLANE_V] + j + l);
            match = 0;
            break;
          }
        }
      }
    }
  }
}

static void testing_decode(aom_codec_ctx_t *encoder, aom_codec_ctx_t *decoder,
                           aom_codec_enc_cfg_t *cfg, unsigned int frame_out,
                           int *mismatch_seen) {
  aom_image_t enc_img, dec_img;
  struct av1_ref_frame ref_enc, ref_dec;

  if (*mismatch_seen) return;

  ref_enc.idx = 0;
  ref_dec.idx = 0;
  if (aom_codec_control(encoder, AV1_GET_REFERENCE, &ref_enc))
    die_codec(encoder, "Failed to get encoder reference frame");
  enc_img = ref_enc.img;
  if (aom_codec_control(decoder, AV1_GET_REFERENCE, &ref_dec))
    die_codec(decoder, "Failed to get decoder reference frame");
  dec_img = ref_dec.img;

  if (!compare_img(&enc_img, &dec_img)) {
    int y[4], u[4], v[4];

    *mismatch_seen = 1;

    find_mismatch(&enc_img, &dec_img, y, u, v);
    printf(
        "Encode/decode mismatch on frame %d at"
        " Y[%d, %d] {%d/%d},"
        " U[%d, %d] {%d/%d},"
        " V[%d, %d] {%d/%d}",
        frame_out, y[0], y[1], y[2], y[3], u[0], u[1], u[2], u[3], v[0], v[1],
        v[2], v[3]);
  }

  aom_img_free(&enc_img);
  aom_img_free(&dec_img);
}

static int encode_frame(aom_codec_ctx_t *ecodec, aom_codec_enc_cfg_t *cfg,
                        aom_image_t *img, unsigned int frame_in,
                        AvxVideoWriter *writer, int test_decode,
                        aom_codec_ctx_t *dcodec, unsigned int *frame_out,
                        int *mismatch_seen) {
  int got_pkts = 0;
  aom_codec_iter_t iter = NULL;
  const aom_codec_cx_pkt_t *pkt = NULL;
  int got_data;
  const aom_codec_err_t res =
      aom_codec_encode(ecodec, img, frame_in, 1, 0, AOM_DL_GOOD_QUALITY);
  if (res != AOM_CODEC_OK) die_codec(ecodec, "Failed to encode frame");

  got_data = 0;

  while ((pkt = aom_codec_get_cx_data(ecodec, &iter)) != NULL) {
    got_pkts = 1;

    if (pkt->kind == AOM_CODEC_CX_FRAME_PKT) {
      const int keyframe = (pkt->data.frame.flags & AOM_FRAME_IS_KEY) != 0;

      if (!(pkt->data.frame.flags & AOM_FRAME_IS_FRAGMENT)) {
        *frame_out += 1;
      }

      if (!aom_video_writer_write_frame(writer, pkt->data.frame.buf,
                                        pkt->data.frame.sz,
                                        pkt->data.frame.pts)) {
        die_codec(ecodec, "Failed to write compressed frame");
      }
      printf(keyframe ? "K" : ".");
      fflush(stdout);
      got_data = 1;

      // Decode 1 frame.
      if (test_decode) {
        if (aom_codec_decode(dcodec, pkt->data.frame.buf,
                             (unsigned int)pkt->data.frame.sz, NULL, 0))
          die_codec(dcodec, "Failed to decode frame.");
      }
    }
  }

  // Mismatch checking
  if (got_data && test_decode) {
    testing_decode(ecodec, dcodec, cfg, *frame_out, mismatch_seen);
  }

  return got_pkts;
}

int main(int argc, char **argv) {
  FILE *infile = NULL;
  // Encoder
  aom_codec_ctx_t ecodec = { 0 };
  aom_codec_enc_cfg_t cfg = { 0 };
  unsigned int frame_in = 0;
  aom_image_t raw;
  aom_codec_err_t res;
  AvxVideoInfo info = { 0 };
  AvxVideoWriter *writer = NULL;
  const AvxInterface *encoder = NULL;

  // Test encoder/decoder mismatch.
  int test_decode = 1;
  // Decoder
  aom_codec_ctx_t dcodec;
  unsigned int frame_out = 0;

  // The frame number to set reference frame on
  int update_frame_num = 0;
  int mismatch_seen = 0;

  const int fps = 30;
  const int bitrate = 500;

  const char *codec_arg = NULL;
  const char *width_arg = NULL;
  const char *height_arg = NULL;
  const char *infile_arg = NULL;
  const char *outfile_arg = NULL;
  int limit = 0;
  exec_name = argv[0];

  if (argc < 7) die("Invalid number of arguments");

  codec_arg = argv[1];
  width_arg = argv[2];
  height_arg = argv[3];
  infile_arg = argv[4];
  outfile_arg = argv[5];

  encoder = get_aom_encoder_by_name(codec_arg);
  if (!encoder) die("Unsupported codec.");

  update_frame_num = atoi(argv[6]);
  // In AV1, the reference buffers (cm->buffer_pool->frame_bufs[i].buf) are
  // allocated while calling aom_codec_encode(), thus, setting reference for
  // 1st frame isn't supported.
  if (update_frame_num <= 1) die("Couldn't parse frame number '%s'\n", argv[6]);

  if (argc > 7) {
    limit = atoi(argv[7]);
    if (update_frame_num > limit)
      die("Update frame number couldn't larger than limit\n");
  }

  info.codec_fourcc = encoder->fourcc;
  info.frame_width = strtol(width_arg, NULL, 0);
  info.frame_height = strtol(height_arg, NULL, 0);
  info.time_base.numerator = 1;
  info.time_base.denominator = fps;

  if (info.frame_width <= 0 || info.frame_height <= 0 ||
      (info.frame_width % 2) != 0 || (info.frame_height % 2) != 0) {
    die("Invalid frame size: %dx%d", info.frame_width, info.frame_height);
  }

  if (!aom_img_alloc(&raw, AOM_IMG_FMT_I420, info.frame_width,
                     info.frame_height, 1)) {
    die("Failed to allocate image.");
  }

  printf("Using %s\n", aom_codec_iface_name(encoder->codec_interface()));

  res = aom_codec_enc_config_default(encoder->codec_interface(), &cfg, 0);
  if (res) die_codec(&ecodec, "Failed to get default codec config.");

  cfg.g_w = info.frame_width;
  cfg.g_h = info.frame_height;
  cfg.g_timebase.num = info.time_base.numerator;
  cfg.g_timebase.den = info.time_base.denominator;
  cfg.rc_target_bitrate = bitrate;
  cfg.g_lag_in_frames = 3;

  writer = aom_video_writer_open(outfile_arg, kContainerIVF, &info);
  if (!writer) die("Failed to open %s for writing.", outfile_arg);

  if (!(infile = fopen(infile_arg, "rb")))
    die("Failed to open %s for reading.", infile_arg);

  if (aom_codec_enc_init(&ecodec, encoder->codec_interface(), &cfg, 0))
    die_codec(&ecodec, "Failed to initialize encoder");

  // Disable alt_ref.
  if (aom_codec_control(&ecodec, AOME_SET_ENABLEAUTOALTREF, 0))
    die_codec(&ecodec, "Failed to set enable auto alt ref");

  if (test_decode) {
    const AvxInterface *decoder = get_aom_decoder_by_name(codec_arg);
    if (aom_codec_dec_init(&dcodec, decoder->codec_interface(), NULL, 0))
      die_codec(&dcodec, "Failed to initialize decoder.");
  }

  // Encode frames.
  while (aom_img_read(&raw, infile)) {
    if (limit && frame_in >= limit) break;
    if (update_frame_num > 1 && frame_out + 1 == update_frame_num) {
      aom_ref_frame_t ref;
      ref.frame_type = AOM_LAST_FRAME;
      ref.img = raw;
      // Set reference frame in encoder.
      if (aom_codec_control(&ecodec, AOM_SET_REFERENCE, &ref))
        die_codec(&ecodec, "Failed to set reference frame");
      printf(" <SET_REF>");

      // If set_reference in decoder is commented out, the enc/dec mismatch
      // would be seen.
      if (test_decode) {
        if (aom_codec_control(&dcodec, AOM_SET_REFERENCE, &ref))
          die_codec(&dcodec, "Failed to set reference frame");
      }
    }

    encode_frame(&ecodec, &cfg, &raw, frame_in, writer, test_decode, &dcodec,
                 &frame_out, &mismatch_seen);
    frame_in++;
    if (mismatch_seen) break;
  }

  // Flush encoder.
  if (!mismatch_seen)
    while (encode_frame(&ecodec, &cfg, NULL, frame_in, writer, test_decode,
                        &dcodec, &frame_out, &mismatch_seen)) {
    }

  printf("\n");
  fclose(infile);
  printf("Processed %d frames.\n", frame_out);

  if (test_decode) {
    if (!mismatch_seen)
      printf("Encoder/decoder results are matching.\n");
    else
      printf("Encoder/decoder results are NOT matching.\n");
  }

  if (test_decode)
    if (aom_codec_destroy(&dcodec))
      die_codec(&dcodec, "Failed to destroy decoder");

  aom_img_free(&raw);
  if (aom_codec_destroy(&ecodec))
    die_codec(&ecodec, "Failed to destroy encoder.");

  aom_video_writer_close(writer);

  return EXIT_SUCCESS;
}
