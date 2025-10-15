#include <string.h>

#include "gpu_codec.h"

GPUEncoder::GPUEncoder(int width, int height, caic_sensor::ImageEncoding type) {
  type_ = type;
  width_ = width;
  height_ = height;

  gpujpeg_set_default_parameters(&param_);
  gpujpeg_image_set_default_parameters(&param_image_);

  param_image_.width = width;
  param_image_.height = height;

  if (type == caic_sensor::ImageEncoding::RGB) {
    // printf("type is RGB.\n");
    param_image_.pixel_format = GPUJPEG_444_U8_P012;
    param_image_.color_space = GPUJPEG_RGB;
  } else if (type == caic_sensor::ImageEncoding::YUV422_YUYV) {
    // printf("type is YUV422_YUYV.\n");
    param_image_.pixel_format = GPUJPEG_422_U8_P1020;
    param_image_.color_space = GPUJPEG_YCBCR_BT709;
  } else {
    printf("GPUEncoder input image format error.\n");
  }

  if ((encoder_ = gpujpeg_encoder_create(0)) == NULL) {
    std::cout << " Encoder init error !" << std::endl;
    exit(-1);
  }
}

GPUEncoder::~GPUEncoder() {
  if (encoder_ != NULL) {
    gpujpeg_encoder_destroy(encoder_);
  }
}

void GPUEncoder::encode(const uint8_t *input, uint8_t **output, int *compress_size) {
  if (type_ == caic_sensor::ImageEncoding::RGB) {
    // printf("type is RGB.\n");
  } else if (type_ == caic_sensor::ImageEncoding::YUV422_YUYV) {
    // printf("type is YUV422_YUYV.\n");
    // for (int i = 0; i < width_ * height_; i++) {
    //   uint8_t temp = input[0 + i * 2];
    //   input[0 + i * 2] = input[1 + i * 2];
    //   input[1 + i * 2] = temp;
    // }
  } else {
    printf("GPUEncoder input image format error.\n");
  }

  gpujpeg_encoder_input_set_image(&encoder_input_, input);
  if (gpujpeg_encoder_encode(encoder_, &param_, &param_image_, &encoder_input_, output,
                             compress_size) != 0) {
    std::cout << "encode error!!!" << std::endl;
    return;
  }
}

GPUDecoder::GPUDecoder(int width, int height, caic_sensor::ImageEncoding type) {
  type_ = type;
  width_ = width;
  height_ = height;

  if ((decoder_ = gpujpeg_decoder_create(0)) == NULL) {
    std::cout << " Decoder init error !" << std::endl;
    exit(-1);
  }

  if (type == caic_sensor::ImageEncoding::RGB) {
    // printf("type is RGB.\n");
    gpujpeg_decoder_set_output_format(decoder_, GPUJPEG_RGB, GPUJPEG_444_U8_P012);
  } else if (type == caic_sensor::ImageEncoding::YUV422_YUYV) {
    // printf("type is YUV422_YUYV.\n");
    gpujpeg_decoder_set_output_format(decoder_, GPUJPEG_YCBCR_BT709, GPUJPEG_422_U8_P1020);
  } else {
    printf("GPUEncoder input image format error.\n");
  }

  gpujpeg_decoder_output_set_default(&decoder_output_);
}

GPUDecoder::~GPUDecoder() {
  if (decoder_ != NULL) {
    gpujpeg_decoder_destroy(decoder_);
  }
}

void GPUDecoder::decode(uint8_t *input, int size, uint8_t *output, int &out_size) {
  if (gpujpeg_decoder_decode(decoder_, input, size, &decoder_output_) != 0) {
    return;
  }
  memcpy(output, decoder_output_.data, decoder_output_.data_size);
  out_size = decoder_output_.data_size;

  if (type_ == caic_sensor::ImageEncoding::RGB) {
    // printf("type is RGB.\n");
  } else if (type_ == caic_sensor::ImageEncoding::YUV422_YUYV) {
    // printf("type is YUV422_YUYV.\n");
    for (int i = 0; i < width_ * height_; i++) {
      uint8_t temp = output[0 + i * 2];
      output[0 + i * 2] = output[1 + i * 2];
      output[1 + i * 2] = temp;
    }
  } else {
    printf("GPUDecoder output image format error.\n");
  }
}
