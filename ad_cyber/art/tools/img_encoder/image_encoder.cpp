// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#include "image_encoder.h"
#include "proto/common/string.pb.h"

caic_sensor::ImageEncoding g_type = caic_sensor::ImageEncoding::YUV422_YUYV;
std::string ch_in;
std::string ch_out;

namespace stoic::app::codec {

void convert_msg(const caic_sensor::Image& msg, std::shared_ptr<ImageH264>& msg_imgh264) {
  int width = msg.width;
  int height = msg.height;
  msg_imgh264->mutable_header()->set_seq(msg.header.seq);
  msg_imgh264->mutable_header()->set_stamp(msg.header.stamp);
  msg_imgh264->mutable_header()->set_frame_id(msg.header.frame_id);
  msg_imgh264->mutable_meta()->set_sensor_timestamp_us(msg.meta.sensor_timestamp_us);
  msg_imgh264->mutable_meta()->set_start_timestamp_us(msg.meta.start_timestamp_us);
  msg_imgh264->mutable_meta()->set_finish_timestamp_us(msg.meta.finish_timestamp_us);
  msg_imgh264->set_available(msg.available);
  msg_imgh264->set_sensor_name(msg.sensor_name);
  msg_imgh264->set_width(width);
  msg_imgh264->set_height(height);
  msg_imgh264->set_codec_size(msg.size);
  msg_imgh264->set_size(msg.size);
  msg_imgh264->set_img_data(msg.p_data, 50000);
}

// int fun_test(int a) { printf("fun_test, a is: %d\n", a); }

void ImageEncoder::convert_msg_h264(const caic_sensor::Image& msg,
                                    std::shared_ptr<ImageH264>& msg_imgh264) {
  int width = msg.width;
  int height = msg.height;
  uint32_t size = msg.size;

  static uint8_t* im_tmp;
  if (!is_init_) {
    im_tmp = (uint8_t*)malloc(size);
    gpu_encoder_ = std::make_shared<GPUEncoder>(width, height, g_type);
    is_init_ = 1;
  }

  // memcpy((void*)im_tmp, (const void*)msg.p_data, size);

  uint8_t* im_h264 = nullptr;
  int compress_size = 0;
  {
    // stoic::Performance perf("encode ", 1);
    gpu_encoder_->encode((const uint8_t*)msg.p_data, &im_h264, &compress_size);
  }
  // printf("[convert_msg_h264] compress_size is: %d\n", compress_size);
  msg_imgh264->mutable_header()->set_seq(msg.header.seq);
  msg_imgh264->mutable_header()->set_stamp(msg.header.stamp);
  msg_imgh264->mutable_header()->set_frame_id(msg.header.frame_id);
  msg_imgh264->mutable_meta()->set_sensor_timestamp_us(msg.meta.sensor_timestamp_us);
  msg_imgh264->mutable_meta()->set_start_timestamp_us(msg.meta.start_timestamp_us);
  msg_imgh264->mutable_meta()->set_finish_timestamp_us(msg.meta.finish_timestamp_us);
  msg_imgh264->set_available(msg.available);
  msg_imgh264->set_encoding((stoic::cm::proto::sensor::ImageCodec_ImageEncoding)msg.encoding);
  msg_imgh264->set_sensor_name(msg.sensor_name);
  msg_imgh264->set_width(width);
  msg_imgh264->set_height(height);
  msg_imgh264->set_codec_size(compress_size);
  msg_imgh264->set_size(msg.size);
  msg_imgh264->set_img_data((void*)im_h264, compress_size);
}

void ImageEncoder::callback(const caic_sensor::Image& msg, const std::string& topic) {
  std::shared_ptr<ImageH264> msg_imgh264_ = std::make_shared<ImageH264>();

  convert_msg_h264(msg, msg_imgh264_);

  printf("recved msg: %s, compresser_size is: %d\n", topic.c_str(), msg_imgh264_->codec_size());

  static cm::Publisher pub1 = nh_ptr_->advertise<ImageH264, false>(ch_out, 1);
  pub1.publish<ImageH264, false>(*(msg_imgh264_.get()));

}

// CompressedImgPtr ImageEncoder::pop() {
//   std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
//   if (frame_buffer_.empty()) {
//     return nullptr;
//   }
//   CompressedImgPtr cimg_ptr = frame_buffer_.front();
//   frame_buffer_.pop();

//   return cimg_ptr;
// }

void ImageEncoder::run() {
  while (cm::ok()) {
    usleep(100000);
  }
}

}  // namespace stoic::app::codec
