// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include "adaptor_base.h"
namespace stoic::cm {

static std::map<std::string, autoplt::tzc::TZCHelper<stoic::cm::proto::sensor::Image>>
    tzcHelper_all;

template <>
struct AdaptorTraits<::caic_sensor::Image, true> {
  using Type = ::caic_sensor::Image;
  using RTType = stoic::cm::proto::sensor::Image;
  using RTTypePtr = std::shared_ptr<stoic::cm::proto::sensor::Image>;

  static void convert(const RTTypePtr& rt_msg, std::shared_ptr<::caic_sensor::Image>& msg) {
    msg->header.seq = rt_msg->header().seq();
    msg->header.stamp = rt_msg->header().stamp();
    msg->header.frame_id = rt_msg->header().frame_id();  // TODO, Segmentation fault in
    msg->meta.sensor_timestamp_us = rt_msg->meta().sensor_timestamp_us();
    msg->meta.start_timestamp_us = rt_msg->meta().start_timestamp_us();
    msg->meta.finish_timestamp_us = rt_msg->meta().finish_timestamp_us();
    msg->available = rt_msg->available();
    msg->sensor_name = rt_msg->sensor_name();
    msg->width = rt_msg->width();
    msg->height = rt_msg->height();
    msg->encoding = (::caic_sensor::ImageEncoding)rt_msg->encoding();
    msg->size = rt_msg->size();
    if (0) {  // 0: SHM
      msg->p_data = (uint8_t*)const_cast<char*>(rt_msg->img_data().c_str());
    }
    // printf("recv convert img, sub msg:: %d\n", msg->width);
  }
  static void convert(const Type& msg, RTTypePtr& rt_msg) {
    auto header = rt_msg->mutable_header();
    header->set_seq(msg.header.seq);
    header->set_stamp(msg.header.stamp);
    header->set_frame_id(msg.header.frame_id);  // TODO, Segmentation fault in runtime.
    rt_msg->mutable_meta()->set_sensor_timestamp_us(msg.meta.sensor_timestamp_us);
    rt_msg->set_sensor_name(msg.sensor_name);
    rt_msg->set_width(msg.width);
    rt_msg->set_height(msg.height);
    rt_msg->set_encoding((stoic::cm::proto::sensor::Image_ImageEncoding)msg.encoding);
    rt_msg->set_size(msg.size);
    if (0) {  // 0: SHM
      rt_msg->set_img_data((const void*)msg.p_data, msg.width * msg.height);
    }
    // printf("send convert img, pub msg:: %d\n", msg.width);
  }
};
template <>
struct AdaptorTraits<::caic_sensor::PointCloudTypeArray, true> {
  using Type = ::caic_sensor::PointCloudTypeArray;
  using RTType = stoic::cm::proto::sensor::PointCloudTypeArray;
  using RTTypePtr = std::shared_ptr<stoic::cm::proto::sensor::PointCloudTypeArray>;

  static void convert(const RTTypePtr& rt_msg, std::shared_ptr<Type>& msg) {
    msg->timestamp = rt_msg->timestamp();
    msg->available = rt_msg->available();
    msg->type = (::caic_sensor::LidarType)rt_msg->type();
    msg->width = rt_msg->width();
    msg->height = rt_msg->height();
    msg->size = rt_msg->size();
    if (1) {//1 use proto SHM; 2 use shm
        int bytes_size = rt_msg->width() * rt_msg->height() * sizeof(caic_sensor::PointXYZIRTL);
        memcpy((char*)msg->points.data(), const_cast<char*>(rt_msg->points_data().c_str()), bytes_size);
    }
    msg->index = rt_msg->index();
    printf("recv convert pointCloud proto to struct, msg->width is: %d\n", msg->width);
  }
  static void convert(const Type& msg, RTTypePtr& rt_msg) {
    rt_msg->set_timestamp(msg.timestamp);
    rt_msg->set_available(msg.available);
    rt_msg->set_type((stoic::cm::proto::sensor::PointCloudTypeArray_LidarType)msg.type);
    rt_msg->set_width(msg.width);
    rt_msg->set_height(msg.height);
    rt_msg->set_size(msg.size);
    rt_msg->set_index(msg.index);
    if (1) { //1 use proto SHM; 2 use shm
        rt_msg->set_points_data((const void*)msg.points.data(), msg.width * msg.height * sizeof(caic_sensor::PointXYZIRTL));
    }
    printf("send convert point cloud, msg.width is: %d, msg.height is: %d\n", msg.width, msg.height);
  }
};

}  // namespace stoic::cm
