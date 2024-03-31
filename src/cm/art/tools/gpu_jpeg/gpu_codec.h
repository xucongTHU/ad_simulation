#include <iostream>

#include "include/gpujpeg.h"

#include "ad_interface.h"

class GPUEncoder {
 public:
  GPUEncoder(int width, int height, caic_sensor::ImageEncoding type);
  ~GPUEncoder();

  void encode(const uint8_t *input, uint8_t **output, int *compress_size);

 private:
  gpujpeg_parameters param_;
  gpujpeg_image_parameters param_image_;

  gpujpeg_encoder *encoder_;
  gpujpeg_encoder_input encoder_input_;

  caic_sensor::ImageEncoding type_;
  int width_;
  int height_;
};

class GPUDecoder {
 public:
  GPUDecoder(int width, int height, caic_sensor::ImageEncoding type);
  ~GPUDecoder();

  void decode(uint8_t *input, int size, uint8_t *output, int &out_size);

 private:
  gpujpeg_decoder_output decoder_output_;

  gpujpeg_decoder *decoder_;

  caic_sensor::ImageEncoding type_;
  int width_;
  int height_;
};
