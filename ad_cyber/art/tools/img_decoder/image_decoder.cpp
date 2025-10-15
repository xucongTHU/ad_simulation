// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#include "image_decoder.h"
#include "proto/common/string.pb.h"

caic_sensor::ImageEncoding g_type = caic_sensor::ImageEncoding::RGB;
std::string ch_in;
std::string ch_out;

namespace stoic::app::codec {

void split_string(const ::std::string& str, char delimiter, ::std::vector<::std::string>* dest) {
  ::std::vector<::std::string> parsed;
  ::std::string::size_type pos = 0;
  while (true) {
    const ::std::string::size_type colon = str.find(delimiter, pos);
    if (colon == ::std::string::npos) {
      parsed.push_back(str.substr(pos));
      break;
    } else {
      parsed.push_back(str.substr(pos, colon - pos));
      pos = colon + 1;
    }
  }
  dest->swap(parsed);
}

bool ImageDecoder::convert_h264msg(
    std::shared_ptr<stoic::cm::proto::sensor::ImageCodec> msg_imgh264, caic_sensor::Image& msg) {
  int width = msg_imgh264->width();
  int height = msg_imgh264->height();
  msg.header.seq = msg_imgh264->mutable_header()->seq();
  msg.header.stamp = msg_imgh264->mutable_header()->stamp();
  msg.header.frame_id = msg_imgh264->mutable_header()->frame_id();
  msg.meta.finish_timestamp_us = msg_imgh264->mutable_meta()->finish_timestamp_us();
  msg.meta.sensor_timestamp_us = msg_imgh264->mutable_meta()->sensor_timestamp_us();
  msg.meta.start_timestamp_us = msg_imgh264->mutable_meta()->start_timestamp_us();
  msg.available = msg_imgh264->available();
  msg.encoding = (caic_sensor::ImageEncoding)msg_imgh264->encoding();
  msg.sensor_name = msg_imgh264->sensor_name();
  msg.width = msg_imgh264->width();
  msg.height = msg_imgh264->height();
  msg.size = msg_imgh264->size();
  int codec_size = (int)msg_imgh264->codec_size();
  int out_size = 0;
  gpu_decoder_->decode((uint8_t*)msg_imgh264->img_data().c_str(), codec_size, msg.p_data, out_size);

  return true;
}

void ImageDecoder::callback(const stoic::cm::proto::sensor::ImageCodec& msg,
                            const std::string& topic) {
  printf("[ImageDecoder::callback] recved msg, topic is: %s\n", topic.c_str());
  if (!is_init_) {
    // init
    gpu_decoder_ =
        std::make_shared<GPUDecoder>(msg.width(), msg.height(), (caic_sensor::ImageEncoding)g_type);
    im_out.p_data = (uint8_t*)malloc(msg.width() * msg.height() * 3);
    is_init_ = 1;
  }
  std::shared_ptr<stoic::cm::proto::sensor::ImageCodec> msg_imgh264 =
      std::make_shared<stoic::cm::proto::sensor::ImageCodec>(msg);
  convert_h264msg(msg_imgh264, im_out);
  // debug
  if (0) {
    static int seq = 0;
    FILE* fp = fopen(("./out_540p_rgb_codec_" + std::to_string(seq++)).c_str(), "wb");
    fwrite(im_out.p_data, 1920 * 1280 * 3, 1, fp);
    fclose(fp);
  }
  // const std::string ch_in_name = topic;
  // std::string ch_out_name;
  // std::vector<::std::string> dest;
  // split_string(ch_in_name, '_', &dest);
  // if (ch_in_name == "/rear/image_raw_compresser") {
  // ch_out_name = "/rear/image_raw";
  // } else {
  // ch_out_name = dest[0] + "_" + dest[1] + "_" + dest[2];
  // }

  static cm::Publisher pub = nh_ptr_->advertise<caic_sensor::Image, true>(ch_out, 1);
  pub.publish<caic_sensor::Image, true>(im_out);
}

}  // namespace stoic::app::codec
