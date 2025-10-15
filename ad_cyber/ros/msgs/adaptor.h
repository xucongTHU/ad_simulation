// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.

#pragma once

#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>

#include "ad_interface.h"
#include "cm/alg/pool/object_pool.hpp"
#include "proto/geometry/vector3.pb.h"
#include "proto/perception/perception_lane.pb.h"
#include "proto/perception/perception_surroundvision.pb.h"
#include "proto/sensor/canbus.pb.h"
#include "ros_custom_mes/CameraData.h"
#include "ros_custom_mes/INSPVAX.h"
#include "ros_custom_mes/InertialSolutionStatus.h"
#include "third_party/apollo/proto/control/control_cmd.pb.h"
#include "third_party/apollo/proto/drivers/chassis.pb.h"
#include "third_party/apollo/proto/drivers/chassis_detail.pb.h"
#include "third_party/apollo/proto/drivers/conti_radar.pb.h"
#include "third_party/apollo/proto/drivers/corner_radar.pb.h"
#include "third_party/apollo/proto/localization/localization.pb.h"
#include "third_party/apollo/proto/planning/planning.pb.h"
#include "third_party/apollo/proto/prediction/prediction_obstacle.pb.h"
#include "third_party/apollo/proto/routing/routing.pb.h"

namespace stoic::cm {

using namespace stoic;  // PRQA S 2522 # Todo: to be solved

template <typename _T, bool _Adaptive = false>
struct AdaptorTraits;

template <>
struct AdaptorTraits<::caic_sensor::Imu, true> {
  using Type = ::caic_sensor::Imu;
  using ROSType = sensor_msgs::Imu;

  static void convert(const ROSType& ros_msg, Type* const msg) {
    msg->header.seq = ros_msg.header.seq;
    msg->header.stamp = ros_msg.header.stamp.sec * 1000000.0 + ros_msg.header.stamp.nsec / 1000.0;
    msg->header.frame_id = "FRD";
    msg->linear_acceleration_info.linear_acceleration.x = ros_msg.linear_acceleration.x;
    msg->linear_acceleration_info.linear_acceleration.y = ros_msg.linear_acceleration.y;
    msg->linear_acceleration_info.linear_acceleration.z = ros_msg.linear_acceleration.z;
    msg->linear_acceleration_info.available = caic_std::LinearAccelerationWithCovariance::VALUE;

    msg->angular_velocity_info.angular_velocity.x = ros_msg.angular_velocity.x;
    msg->angular_velocity_info.angular_velocity.y = ros_msg.angular_velocity.y;
    msg->angular_velocity_info.angular_velocity.z = ros_msg.angular_velocity.z;
    msg->angular_velocity_info.available = caic_std::AngularVelocityWithCovariance::VALUE;

    msg->available = caic_sensor::Imu::IMU_LINEAR_ACCELERATION_INFO |
                     caic_sensor::Imu::IMU_ANGULAR_VELOCITY_INFO;
  }

  static void convert(const Type&, ROSType*) {
    std::cout << "Imu deserialize not implemented." << std::endl;
  }
};

template <>
struct AdaptorTraits<caic_sensor::caic_ins::Odometry, true> {
  using Type = caic_sensor::caic_ins::Odometry;
  using ROSType = nav_msgs::Odometry;

  static void convert(const ROSType& ros_msg, Type* const msg) {
    msg->header.seq = ros_msg.header.seq;
    msg->header.stamp = ros_msg.header.stamp.sec * 1000000.0 + ros_msg.header.stamp.nsec / 1000.0;
    msg->header.frame_id = ros_msg.header.frame_id;
    msg->child_frame_id = ros_msg.child_frame_id;
    msg->pose_with_covariance.position.x = ros_msg.pose.pose.position.x;
    msg->pose_with_covariance.position.y = ros_msg.pose.pose.position.y;
    msg->pose_with_covariance.position.z = ros_msg.pose.pose.position.z;
    msg->pose_with_covariance.quaternion.x = ros_msg.pose.pose.orientation.x;
    msg->pose_with_covariance.quaternion.y = ros_msg.pose.pose.orientation.y;
    msg->pose_with_covariance.quaternion.z = ros_msg.pose.pose.orientation.z;
    msg->pose_with_covariance.quaternion.w = ros_msg.pose.pose.orientation.w;
    for (size_t i = 0; i < 36; ++i) {
      msg->pose_with_covariance.covariance.data.at(i) = ros_msg.pose.covariance[i];
    }
    msg->pose_with_covariance.available = caic_std::PoseWithCovariance::POSITION |
                                          caic_std::PoseWithCovariance::ORIENTATION |
                                          caic_std::PoseWithCovariance::COVARINACE;
    msg->twist_with_covariance.linear.x = ros_msg.twist.twist.linear.x;
    msg->twist_with_covariance.linear.y = ros_msg.twist.twist.linear.y;
    msg->twist_with_covariance.linear.z = ros_msg.twist.twist.linear.z;
    msg->twist_with_covariance.angular.x = ros_msg.twist.twist.angular.x;
    msg->twist_with_covariance.angular.y = ros_msg.twist.twist.angular.y;
    msg->twist_with_covariance.angular.z = ros_msg.twist.twist.angular.z;
    for (size_t i = 0; i < 36; ++i) {
      msg->twist_with_covariance.covariance.data.at(i) = ros_msg.twist.covariance[i];
    }
    msg->twist_with_covariance.available =
        caic_std::TwistWithCovariance::LINEAR | caic_std::TwistWithCovariance::COVARINACE;

    msg->available = caic_sensor::caic_ins::Odometry::ODOMETRY_CHILD_FRAME_ID |
                     caic_sensor::caic_ins::Odometry::ODOMETRY_POSE_WITH_COVARIANCE |
                     caic_sensor::caic_ins::Odometry::ODOMETRY_TWIST_WITH_COVARIANCE;
  }

  static void convert(const Type&, ROSType*) {
    std::cout << "GNSS deserialize not implemented." << std::endl;
  }
};

template <>
struct AdaptorTraits<::caic_sensor::caic_ins::CorrImu, true> {
  using Type = ::caic_sensor::caic_ins::CorrImu;
  using ROSType = sensor_msgs::Imu;

  static void convert(const ROSType& ros_msg, Type* const msg) {
    msg->header.seq = ros_msg.header.seq;
    msg->header.stamp = ros_msg.header.stamp.sec * 1000000.0 + ros_msg.header.stamp.nsec / 1000.0;
    msg->header.frame_id = "ENU";

    msg->orientation_info.quaternion.w = ros_msg.orientation.w;
    msg->orientation_info.quaternion.x = ros_msg.orientation.x;
    msg->orientation_info.quaternion.y = ros_msg.orientation.y;
    msg->orientation_info.quaternion.z = ros_msg.orientation.z;
    for (size_t i = 0; i < 9; ++i) {  // PRQA S 4400 # TODO: to be solved
      msg->orientation_info.quaternion_covariance.data.at(i) = ros_msg.orientation_covariance[i];
    }
    msg->orientation_info.available = caic_std::OrientationWithCovariance::VALUE |
                                      caic_std::OrientationWithCovariance::QUATERNION_COVARIANCE;

    msg->linear_acceleration_info.linear_acceleration.x = ros_msg.linear_acceleration.x;
    msg->linear_acceleration_info.linear_acceleration.y = ros_msg.linear_acceleration.y;
    msg->linear_acceleration_info.linear_acceleration.z = ros_msg.linear_acceleration.z;
    msg->linear_acceleration_info.available = caic_std::LinearAccelerationWithCovariance::VALUE;

    msg->angular_velocity_info.angular_velocity.x = ros_msg.angular_velocity.x;
    msg->angular_velocity_info.angular_velocity.y = ros_msg.angular_velocity.y;
    msg->angular_velocity_info.angular_velocity.z = ros_msg.angular_velocity.z;
    msg->angular_velocity_info.available = caic_std::AngularVelocityWithCovariance::VALUE;

    msg->available = caic_sensor::caic_ins::CorrImu::CORR_IMU_ORIENTATION_INFO |
                     caic_sensor::caic_ins::CorrImu::CORR_IMU_LINEAR_ACCELERATION_INFO |
                     caic_sensor::caic_ins::CorrImu::CORR_IMU_ANGULAR_VELOCITY_INFO;
  }

  static void convert(const Type&, ROSType*) {
    std::cout << "corrimu deserialize not implemented." << std::endl;
  }
};

template <>
struct AdaptorTraits<::caic_sensor::caic_ins::Inspavx, true> {
  using Type = ::caic_sensor::caic_ins::Inspavx;
  using ROSType = ::ros_custom_mes::INSPVAX;

  static void convert(const ROSType& ros_msg, Type* msg) {
    msg->header.seq = ros_msg.header.seq;
    msg->header.stamp = ros_msg.header.stamp.sec * 1000000.0 + ros_msg.header.stamp.nsec / 1000.0;
    msg->header.frame_id = "ENU";

    msg->status = (caic_sensor::caic_ins::InsStatus)(ros_msg.ins_status.status);
    msg->pos_type = (caic_sensor::caic_ins::InsPoseType)(ros_msg.pos_type.type);
    msg->ext_sol_status = (ros_msg.ext_sol_status.status);

    msg->position.latitude_stdev = ros_msg.latitude_stdev;
    msg->position.longitude_stdev = ros_msg.longitude_stdev;
    msg->orientation.azimuth_stdev = ros_msg.azimuth_stdev;

    msg->available = caic_sensor::caic_ins::Inspavx::INSPV_X_INS_STATUS |
                     caic_sensor::caic_ins::Inspavx::INSPV_X_POS_TYPE |
                     caic_sensor::caic_ins::Inspavx::INSPV_X_EXT_SOL_STATUS |
                     caic_sensor::caic_ins::Inspavx::INSPV_X_POSITION |
                     caic_sensor::caic_ins::Inspavx::INSPV_X_ORIENATION;

    std::cout << "rec ap msg time:" << msg->header.stamp
              << ",inspavx solution status: " << (uint32_t)msg->ext_sol_status
              << ",pos type: " << (int)msg->pos_type << std::endl;
  }

  static void convert(const Type&, ROSType*) {
    std::cout << "inspvax deserialize not implemented." << std::endl;
  }
};

template <>
struct AdaptorTraits<::caic_localization::LocalizationEstimation, true> {
  using Type = ::caic_localization::LocalizationEstimation;
  using ProtoType = ::apollo::localization::LocalizationEstimate;
  using ROSType = std_msgs::String;

  static void convert(const ROSType& ros_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ros_msg.data);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, ROSType* ros_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ros_msg->data);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    msg->header.stamp = proto_msg.header().timestamp_sec() * 1000000.0;
    msg->msf_status.gnsspos_position_type = static_cast<caic_localization::GnssPositionType>(
        proto_msg.msf_status().gnsspos_position_type());
    msg->pose.pose_info.position.x = proto_msg.pose().position().x();
    msg->pose.pose_info.position.y = proto_msg.pose().position().y();
    msg->pose.pose_info.position.z = proto_msg.pose().position().z();
    msg->pose.pose_info.quaternion.x = proto_msg.pose().orientation().qx();
    msg->pose.pose_info.quaternion.y = proto_msg.pose().orientation().qy();
    msg->pose.pose_info.quaternion.z = proto_msg.pose().orientation().qz();
    msg->pose.pose_info.quaternion.w = proto_msg.pose().orientation().qw();
    msg->pose.heading = proto_msg.pose().heading();
    msg->pose.linear_velocity_info.linear_velocity.x = proto_msg.pose().linear_velocity().x();
    msg->pose.linear_velocity_info.linear_velocity.y = proto_msg.pose().linear_velocity().y();
    msg->pose.linear_velocity_info.linear_velocity.z = proto_msg.pose().linear_velocity().z();
    msg->pose.angle.roll = proto_msg.pose().euler_angles().x();
    msg->pose.angle.pitch = proto_msg.pose().euler_angles().y();
    msg->pose.angle.yaw = proto_msg.pose().euler_angles().z();
    msg->pose.linear_acceleration_info.linear_acceleration.x =
        proto_msg.pose().linear_acceleration().x();
    msg->pose.linear_acceleration_info.linear_acceleration.y =
        proto_msg.pose().linear_acceleration().y();
    msg->pose.linear_acceleration_info.linear_acceleration.z =
        proto_msg.pose().linear_acceleration().z();
    msg->pose.angular_velocity_info.angular_velocity.x = proto_msg.pose().angular_velocity().x();
    msg->pose.angular_velocity_info.angular_velocity.y = proto_msg.pose().angular_velocity().y();
    msg->pose.angular_velocity_info.angular_velocity.z = proto_msg.pose().angular_velocity().z();
    msg->pose.angular_velocity_vrf_info.angular_velocity.x =
        proto_msg.pose().angular_velocity_vrf().x();
    msg->pose.angular_velocity_vrf_info.angular_velocity.y =
        proto_msg.pose().angular_velocity_vrf().y();
    msg->pose.angular_velocity_vrf_info.angular_velocity.z =
        proto_msg.pose().angular_velocity_vrf().z();
    msg->pose.pose_info.covariance.data[0] = pow(proto_msg.uncertainty().position_std_dev().x(), 2);
    msg->pose.pose_info.covariance.data[7] = pow(proto_msg.uncertainty().position_std_dev().y(), 2);
    msg->pose.pose_info.covariance.data[14] =
        pow(proto_msg.uncertainty().position_std_dev().z(), 2);
    msg->pose.pose_info.covariance.data[28] =
        pow(proto_msg.uncertainty().orientation_std_dev().x(), 2);
    msg->pose.pose_info.covariance.data[21] =
        pow(proto_msg.uncertainty().orientation_std_dev().y(), 2);
    msg->pose.pose_info.covariance.data[35] =
        pow(proto_msg.uncertainty().orientation_std_dev().z(), 2);
    msg->pose.linear_velocity_info.linear_velocity_covariance.data[0] =
        pow(proto_msg.uncertainty().linear_velocity_std_dev().x(), 2);
    msg->pose.linear_velocity_info.linear_velocity_covariance.data[4] =
        pow(proto_msg.uncertainty().linear_velocity_std_dev().y(), 2);
    msg->pose.linear_velocity_info.linear_velocity_covariance.data[8] =
        pow(proto_msg.uncertainty().linear_velocity_std_dev().z(), 2);
    msg->pose.linear_acceleration_info.linear_acceleration_covariance.data[0] =
        pow(proto_msg.uncertainty().linear_acceleration_std_dev().x(), 2);
    msg->pose.linear_acceleration_info.linear_acceleration_covariance.data[4] =
        pow(proto_msg.uncertainty().linear_acceleration_std_dev().y(), 2);
    msg->pose.linear_acceleration_info.linear_acceleration_covariance.data[8] =
        pow(proto_msg.uncertainty().linear_acceleration_std_dev().z(), 2);
    msg->pose.angular_velocity_info.angular_velocity_covariance.data[0] =
        pow(proto_msg.uncertainty().angular_velocity_std_dev().x(), 2);
    msg->pose.angular_velocity_info.angular_velocity_covariance.data[4] =
        pow(proto_msg.uncertainty().angular_velocity_std_dev().y(), 2);
    msg->pose.angular_velocity_info.angular_velocity_covariance.data[8] =
        pow(proto_msg.uncertainty().angular_velocity_std_dev().z(), 2);
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    proto_msg->Clear();
    // header
    static long long localization_seq_num_ = 0;
    std::string module_name = "localization_proto";
    auto* header = proto_msg->mutable_header();
    double timestamp = msg.header.stamp;
    header->set_module_name(module_name);
    header->set_timestamp_sec(timestamp * 1.0 / 1000000.0);
    header->set_sequence_num(static_cast<unsigned int>(++localization_seq_num_));

    proto_msg->set_measurement_time(timestamp * 1.0 / 1000000.0);
    auto mutable_msf_status = proto_msg->mutable_msf_status();
    mutable_msf_status->set_gnsspos_position_type(
        static_cast<apollo::localization::GnssPositionType>(msg.msf_status.gnsspos_position_type));

    auto mutable_pose = proto_msg->mutable_pose();
    // position
    mutable_pose->mutable_position()->set_x(msg.pose.pose_info.position.x);
    mutable_pose->mutable_position()->set_y(msg.pose.pose_info.position.y);
    mutable_pose->mutable_position()->set_z(msg.pose.pose_info.position.z);

    // orientation
    mutable_pose->mutable_orientation()->set_qx(msg.pose.pose_info.quaternion.x);
    mutable_pose->mutable_orientation()->set_qy(msg.pose.pose_info.quaternion.y);
    mutable_pose->mutable_orientation()->set_qz(msg.pose.pose_info.quaternion.z);
    mutable_pose->mutable_orientation()->set_qw(msg.pose.pose_info.quaternion.w);

    double heading = msg.pose.heading;

    mutable_pose->set_heading(heading);

    // linear velocity
    mutable_pose->mutable_linear_velocity()->set_x(msg.pose.linear_velocity_info.linear_velocity.x);
    mutable_pose->mutable_linear_velocity()->set_y(msg.pose.linear_velocity_info.linear_velocity.y);
    mutable_pose->mutable_linear_velocity()->set_z(msg.pose.linear_velocity_info.linear_velocity.z);

    mutable_pose->mutable_euler_angles()->set_x(msg.pose.angle.roll);
    mutable_pose->mutable_euler_angles()->set_y(msg.pose.angle.pitch);
    mutable_pose->mutable_euler_angles()->set_z(msg.pose.angle.yaw);

    mutable_pose->mutable_linear_acceleration()->set_x(
        msg.pose.linear_acceleration_info.linear_acceleration.x);
    mutable_pose->mutable_linear_acceleration()->set_y(
        msg.pose.linear_acceleration_info.linear_acceleration.y);
    mutable_pose->mutable_linear_acceleration()->set_z(
        msg.pose.linear_acceleration_info.linear_acceleration.z);
    mutable_pose->mutable_linear_acceleration_vrf()->set_x(
        msg.pose.linear_acceleration_vrf_info.linear_acceleration.x);
    mutable_pose->mutable_linear_acceleration_vrf()->set_y(
        msg.pose.linear_acceleration_vrf_info.linear_acceleration.y);
    mutable_pose->mutable_linear_acceleration_vrf()->set_z(
        msg.pose.linear_acceleration_vrf_info.linear_acceleration.z);

    // angular_velocity
    mutable_pose->mutable_angular_velocity()->set_x(
        msg.pose.angular_velocity_info.angular_velocity.x);
    mutable_pose->mutable_angular_velocity()->set_y(
        msg.pose.angular_velocity_info.angular_velocity.y);
    mutable_pose->mutable_angular_velocity()->set_z(
        msg.pose.angular_velocity_info.angular_velocity.z);

    mutable_pose->mutable_angular_velocity_vrf()->set_x(
        msg.pose.angular_velocity_vrf_info.angular_velocity.x);
    mutable_pose->mutable_angular_velocity_vrf()->set_y(
        msg.pose.angular_velocity_vrf_info.angular_velocity.y);
    mutable_pose->mutable_angular_velocity_vrf()->set_z(
        msg.pose.angular_velocity_vrf_info.angular_velocity.z);

    auto mutable_uncertainty = proto_msg->mutable_uncertainty();
    // position_std_dev
    mutable_uncertainty->mutable_position_std_dev()->set_x(
        sqrt(msg.pose.pose_info.covariance.data[0]));
    mutable_uncertainty->mutable_position_std_dev()->set_y(
        sqrt(msg.pose.pose_info.covariance.data[7]));
    mutable_uncertainty->mutable_position_std_dev()->set_z(
        sqrt(msg.pose.pose_info.covariance.data[14]));

    // orientation_std_dev
    mutable_uncertainty->mutable_orientation_std_dev()->set_x(
        sqrt(msg.pose.pose_info.covariance.data[28]));
    mutable_uncertainty->mutable_orientation_std_dev()->set_y(
        sqrt(msg.pose.pose_info.covariance.data[21]));
    mutable_uncertainty->mutable_orientation_std_dev()->set_z(
        sqrt(msg.pose.pose_info.covariance.data[35]));

    // linear_velocity_std_dev
    mutable_uncertainty->mutable_linear_velocity_std_dev()->set_x(
        sqrt(msg.pose.linear_velocity_info.linear_velocity_covariance.data[0]));
    mutable_uncertainty->mutable_linear_velocity_std_dev()->set_y(
        sqrt(msg.pose.linear_velocity_info.linear_velocity_covariance.data[4]));
    mutable_uncertainty->mutable_linear_velocity_std_dev()->set_z(
        sqrt(msg.pose.linear_velocity_info.linear_velocity_covariance.data[8]));

    // linear_acceleration_std_dev
    mutable_uncertainty->mutable_linear_acceleration_std_dev()->set_x(
        sqrt(msg.pose.linear_acceleration_info.linear_acceleration_covariance.data[0]));
    mutable_uncertainty->mutable_linear_acceleration_std_dev()->set_y(
        sqrt(msg.pose.linear_acceleration_info.linear_acceleration_covariance.data[4]));
    mutable_uncertainty->mutable_linear_acceleration_std_dev()->set_z(
        sqrt(msg.pose.linear_acceleration_info.linear_acceleration_covariance.data[8]));

    //
    mutable_uncertainty->mutable_angular_velocity_std_dev()->set_x(
        sqrt(msg.pose.angular_velocity_info.angular_velocity_covariance.data[0]));
    mutable_uncertainty->mutable_angular_velocity_std_dev()->set_y(
        sqrt(msg.pose.angular_velocity_info.angular_velocity_covariance.data[4]));
    mutable_uncertainty->mutable_angular_velocity_std_dev()->set_z(
        sqrt(msg.pose.angular_velocity_info.angular_velocity_covariance.data[8]));

    std::string localization_state_message = "proto_msg state is good";
    proto_msg->set_state_message(localization_state_message);
  }
};

// lane
template <>
struct AdaptorTraits<::caic_perception::PerceptionLanes, true> {
  using Type = ::caic_perception::PerceptionLanes;
  using ProtoType = stoic::cm::proto::perception::PerceptionLanes;
  using ROSType = std_msgs::String;

  static void convert(const ROSType& ros_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ros_msg.data);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, ROSType* ros_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ros_msg->data);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    // msg->header.stamp = proto_msg.header().timestamp_sec() * 1000000l;
    // msg->meta.sensor_timestamp_us = proto_msg.meta().sensor_timestamp_us();
    // msg->meta.start_timestamp_us = proto_msg.meta().start_timestamp_us();
    // msg->meta.finish_timestamp_us = proto_msg.meta().finish_timestamp_us();
    // msg->size = proto_msg.size();

    // for (int i = 0; i < proto_msg.lanes_size(); ++i) {
    //   auto proto_lane = proto_msg.lanes(i);
    //   caic_perception::PerceptionLane lane;
    //   lane.avaliable = proto_lane.avaliable();
    //   lane.id = proto_lane.id();
    //   lane.color = static_cast<caic_perception::LaneColor>(proto_lane.color());
    //   lane.type = static_cast<caic_perception::LaneType>(proto_lane.type());
    //   lane.index = static_cast<caic_perception::LaneIndex>(proto_lane.index());
    //   lane.score = proto_lane.score();

    //   auto poly = proto_lane.poly();
    //   lane.poly.start_x = poly.start_x();
    //   lane.poly.end_x = poly.end_x();
    //   lane.poly.c0 = poly.c0();
    //   lane.poly.c1 = poly.c1();
    //   lane.poly.c2 = poly.c2();
    //   lane.poly.c3 = poly.c3();

    //   for (int j = 0; j < proto_lane.points_size(); ++j) {
    //     caic_std::Point2d pt;
    //     auto proto_point = proto_lane.points(j);
    //     pt.x = proto_point.x();
    //     pt.y = proto_point.y();
    //     lane.points.push_back(pt);
    //   }

    //   lane.is_predict = proto_lane.is_predict();
    //   lane.width = proto_lane.width();
    //   lane.stop_line_type =
    //   static_cast<caic_perception::StopLineType>(proto_lane.stop_line_type());

    //   msg->lanes.push_back(lane);
    // }

    // for (int i = 0; i < proto_msg.stop_lines_size(); ++i) {
    //   auto proto_stopline = proto_msg.stop_lines(i);
    //   caic_perception::PerceptionStopLine stopline;
    //   stopline.avaliable = proto_stopline.avaliable();
    //   stopline.id = proto_stopline.id();
    //   stopline.score = proto_stopline.score();
    //   stopline.is_predict = proto_stopline.is_predict();
    //   stopline.width = proto_stopline.width();
    //   stopline.left_end_point.x = proto_stopline.left_end_point().x();
    //   stopline.left_end_point.y = proto_stopline.left_end_point().y();
    //   stopline.right_end_point.x = proto_stopline.right_end_point().x();
    //   stopline.right_end_point.y = proto_stopline.right_end_point().y();

    //   auto poly = proto_stopline.poly();
    //   stopline.poly.start_y = poly.start_y();
    //   stopline.poly.end_y = poly.end_y();
    //   stopline.poly.c0 = poly.c0();
    //   stopline.poly.c1 = poly.c1();
    //   stopline.poly.c2 = poly.c2();
    //   stopline.poly.c3 = poly.c3();

    //   for (int j = 0; j < proto_stopline.points_size(); ++j) {
    //     caic_std::Point2d pt;
    //     auto proto_point = proto_stopline.points(j);
    //     pt.x = proto_point.x();
    //     pt.y = proto_point.y();
    //     stopline.points.push_back(pt);
    //   }

    //   stopline.type = static_cast<caic_perception::StopLineType>(proto_stopline.type());
    //   msg->stop_lines.push_back(stopline);
    // }
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    // auto header = proto_msg->mutable_header();
    // header->set_timestamp_sec(msg.header.stamp * 1.0 / 1000000l);

    // auto meta = proto_msg->mutable_meta();
    // meta->set_sensor_timestamp_us(msg.meta.sensor_timestamp_us);
    // meta->set_start_timestamp_us(msg.meta.start_timestamp_us);
    // meta->set_finish_timestamp_us(msg.meta.finish_timestamp_us);

    // proto_msg->set_size(msg.size);

    // for (size_t i = 0; i < msg.lanes.size(); ++i) {
    //   auto lane = msg.lanes[i];
    //   auto proto_lane = proto_msg->add_lanes();
    //   proto_lane->set_avaliable(lane.avaliable);
    //   proto_lane->set_id(lane.id);
    //   proto_lane->set_score(lane.score);
    //   proto_lane->set_is_predict(lane.is_predict);
    //   proto_lane->set_width(lane.width);
    //   proto_lane->set_color(static_cast<apollo::perception::vision::LaneColor>(lane.color));
    //   proto_lane->set_type(static_cast<apollo::perception::vision::LaneType>(lane.type));
    //   proto_lane->set_index(static_cast<apollo::perception::vision::LaneIndex>(lane.index));
    //   auto poly = proto_lane->mutable_poly();
    //   poly->set_start_x(lane.poly.start_x);
    //   poly->set_end_x(lane.poly.end_x);
    //   poly->set_c0(lane.poly.c0);
    //   poly->set_c1(lane.poly.c1);
    //   poly->set_c2(lane.poly.c2);
    //   poly->set_c3(lane.poly.c3);

    //   for (size_t j = 0; j < lane.points.size(); ++i) {
    //     auto pt = proto_lane->add_points();
    //     pt->set_x(lane.points[j].x);
    //     pt->set_y(lane.points[j].y);
    //   }

    //   proto_lane->set_stop_line_type(
    //       static_cast<apollo::perception::vision::StopLineType>(lane.stop_line_type));
    // }

    // for (size_t i = 0; i < msg.stop_lines.size(); ++i) {
    //   auto stop_line = msg.stop_lines[i];
    //   auto proto_stopline = proto_msg->add_stop_lines();

    //   proto_stopline->set_avaliable(stop_line.avaliable);
    //   proto_stopline->set_id(stop_line.id);
    //   proto_stopline->set_score(stop_line.score);
    //   proto_stopline->set_is_predict(stop_line.is_predict);
    //   proto_stopline->set_width(stop_line.width);

    //   auto left_end_point = proto_stopline->mutable_left_end_point();
    //   left_end_point->set_x(stop_line.left_end_point.x);
    //   left_end_point->set_y(stop_line.left_end_point.y);
    //   auto right_end_point = proto_stopline->mutable_right_end_point();
    //   right_end_point->set_x(stop_line.right_end_point.x);
    //   right_end_point->set_y(stop_line.right_end_point.y);

    //   auto poly = proto_stopline->mutable_poly();
    //   poly->set_start_y(stop_line.poly.start_y);
    //   poly->set_end_y(stop_line.poly.end_y);
    //   poly->set_c0(stop_line.poly.c0);
    //   poly->set_c1(stop_line.poly.c1);
    //   poly->set_c2(stop_line.poly.c2);
    //   poly->set_c3(stop_line.poly.c3);

    //   for (size_t j = 0; j < stop_line.points.size(); ++i) {
    //     auto pt = proto_stopline->add_points();
    //     pt->set_x(stop_line.points[j].x);
    //     pt->set_y(stop_line.points[j].y);
    //   }

    //   proto_stopline->set_type(
    //       static_cast<apollo::perception::vision::StopLineType>(stop_line.type));
    // }
  }
};

// surround
template <>
struct AdaptorTraits<::caic_perception::PerceptionSurround, true> {
  using Type = ::caic_perception::PerceptionSurround;
  using ProtoType = stoic::cm::proto::perception::PerceptionSurround;
  using ROSType = std_msgs::String;

  static void convert(const ROSType& ros_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ros_msg.data);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, ROSType* ros_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ros_msg->data);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    // msg->header.stamp = proto_msg.header().timestamp_sec() * 1000000l;

    // // parking space
    // for (int i = 0; i < proto_msg.parking_space_frame().parking_space_size(); ++i) {
    //   caic_perception::ParkingSpace parkingspace;

    //   auto out_parking = proto_msg.parking_space_frame().parking_space(i);
    //   parkingspace.timestamp = out_parking.timestamp() * 1000000l;
    //   parkingspace.track_id = out_parking.track_id();
    //   parkingspace.lost_age = out_parking.lost_age();

    //   caic_perception::SpaceInfo space_vcs, space_ipm;
    //   space_vcs.parkingspace_left_up_point.x = out_parking.space_vcs().x1();
    //   space_vcs.parkingspace_left_up_point.y = out_parking.space_vcs().y1();
    //   space_vcs.parkingspace_right_up_point.x = out_parking.space_vcs().x2();
    //   space_vcs.parkingspace_right_up_point.y = out_parking.space_vcs().y2();
    //   space_vcs.parkingspace_left_down_point.x = out_parking.space_vcs().x3();
    //   space_vcs.parkingspace_left_down_point.y = out_parking.space_vcs().y3();
    //   space_vcs.parkingspace_right_down_point.x = out_parking.space_vcs().x4();
    //   space_vcs.parkingspace_right_down_point.y = out_parking.space_vcs().y4();
    //   space_vcs.chock_left_point.x = out_parking.space_vcs().chock_x1();
    //   space_vcs.chock_left_point.y = out_parking.space_vcs().chock_y1();
    //   space_vcs.chock_right_point.x = out_parking.space_vcs().chock_x2();
    //   space_vcs.chock_right_point.y = out_parking.space_vcs().chock_y2();
    //   space_vcs.angle_left = out_parking.space_vcs().angle1();
    //   space_vcs.angle_right = out_parking.space_vcs().angle2();
    //   space_vcs.left_up_point_occ_by_statuscar =
    //       out_parking.space_vcs().left_up_point_occ_statuscar();
    //   space_vcs.left_up_point_occ_by_obstacle =
    //       out_parking.space_vcs().left_up_point_occ_by_obstacle();
    //   space_vcs.left_up_point_camera_source =
    //       static_cast<uint8_t>(out_parking.space_vcs().left_up_point_camera_source());
    //   space_vcs.right_up_point_occ_by_statuscar =
    //       out_parking.space_vcs().right_up_point_occ_statuscar();
    //   space_vcs.right_up_point_occ_by_obstacle =
    //       out_parking.space_vcs().right_up_point_occ_by_obstacle();
    //   space_vcs.right_up_point_camera_source =
    //       static_cast<uint8_t>(out_parking.space_vcs().right_up_point_camera_source());

    //   space_ipm.parkingspace_left_up_point.x = out_parking.space_ipm().x1();
    //   space_ipm.parkingspace_left_up_point.y = out_parking.space_ipm().y1();
    //   space_ipm.parkingspace_right_up_point.x = out_parking.space_ipm().x2();
    //   space_ipm.parkingspace_right_up_point.y = out_parking.space_ipm().y2();
    //   space_ipm.parkingspace_left_down_point.x = out_parking.space_ipm().x3();
    //   space_ipm.parkingspace_left_down_point.y = out_parking.space_ipm().y3();
    //   space_ipm.parkingspace_right_down_point.x = out_parking.space_ipm().x4();
    //   space_ipm.parkingspace_right_down_point.y = out_parking.space_ipm().y4();
    //   space_ipm.chock_left_point.x = out_parking.space_ipm().chock_x1();
    //   space_ipm.chock_left_point.y = out_parking.space_ipm().chock_y1();
    //   space_ipm.chock_right_point.x = out_parking.space_ipm().chock_x2();
    //   space_ipm.chock_right_point.y = out_parking.space_ipm().chock_y2();
    //   space_ipm.angle_left = out_parking.space_ipm().angle1();
    //   space_ipm.angle_right = out_parking.space_ipm().angle2();
    //   space_ipm.left_up_point_occ_by_statuscar =
    //       out_parking.space_ipm().left_up_point_occ_statuscar();
    //   space_ipm.left_up_point_occ_by_obstacle =
    //       out_parking.space_ipm().left_up_point_occ_by_obstacle();
    //   space_ipm.left_up_point_camera_source =
    //       static_cast<uint8_t>(out_parking.space_ipm().left_up_point_camera_source());
    //   space_ipm.right_up_point_occ_by_statuscar =
    //       out_parking.space_ipm().right_up_point_occ_statuscar();
    //   space_ipm.right_up_point_occ_by_obstacle =
    //       out_parking.space_ipm().right_up_point_occ_by_obstacle();
    //   space_ipm.right_up_point_camera_source =
    //       static_cast<uint8_t>(out_parking.space_ipm().right_up_point_camera_source());

    //   parkingspace.space_vcs = space_vcs;
    //   parkingspace.space_ipm = space_ipm;
    //   parkingspace.score = out_parking.score();
    //   parkingspace.type = static_cast<uint8_t>(out_parking.type());
    //   parkingspace.occu = static_cast<uint8_t>(out_parking.occu());
    //   parkingspace.psd_id = out_parking.psd_id();

    //   msg->parking_space_result.parking_spaces.push_back(parkingspace);
    // }

    // // freespace
    // for (int i = 0; i < proto_msg.free_space_objects().free_space_points_size(); ++i) {
    //   auto pt = proto_msg.free_space_objects().free_space_points(i);
    //   caic_perception::FreespacePoint freespace_pt;
    //   freespace_pt.pt.ipm_point.x = pt.ipm_x();
    //   freespace_pt.pt.ipm_point.y = pt.ipm_y();
    //   freespace_pt.pt.ipm_point.x = pt.vcs_x();
    //   freespace_pt.pt.ipm_point.y = pt.vcs_y();
    //   freespace_pt.type = static_cast<uint8_t>(pt.type());
    //   msg->freespace_result.free_space_points.push_back(freespace_pt);
    // }

    // // lanemark
    // for (int i = 0; i < proto_msg.result_lanemark().lanemark_objects_size(); ++i) {
    //   caic_perception::LanemarkObject lanemark;
    //   auto out_lanemark = proto_msg.result_lanemark().lanemark_objects(i);
    //   lanemark.mark_id = out_lanemark.mark_id();

    //   for (int j = 0; j < out_lanemark.lanemark_points_size(); ++j) {
    //     caic_perception::LanemarkPoint lanemark_pt;

    //     auto out_lanemark_point = out_lanemark.lanemark_points(j);
    //     lanemark_pt.pt.ipm_point.x = out_lanemark_point.pt_ipm_x();
    //     lanemark_pt.pt.ipm_point.y = out_lanemark_point.pt_ipm_y();
    //     lanemark_pt.pt.vcs_point.x = out_lanemark_point.pt_vcs_x();
    //     lanemark_pt.pt.vcs_point.y = out_lanemark_point.pt_vcs_y();
    //     lanemark_pt.score = out_lanemark_point.score();
    //     lanemark_pt.type = static_cast<uint8_t>(out_lanemark_point.type());

    //     lanemark.lanemark_points.push_back(lanemark_pt);
    //   }

    //   lanemark.lanemark_box.pt.ipm_point.x = out_lanemark.lanemark_box().pt_ipm_x();
    //   lanemark.lanemark_box.pt.ipm_point.y = out_lanemark.lanemark_box().pt_ipm_y();
    //   lanemark.lanemark_box.pt.vcs_point.x = out_lanemark.lanemark_box().pt_vcs_x();
    //   lanemark.lanemark_box.pt.vcs_point.y = out_lanemark.lanemark_box().pt_vcs_y();
    //   lanemark.lanemark_box.box_w = out_lanemark.lanemark_box().box_w();
    //   lanemark.lanemark_box.box_h = out_lanemark.lanemark_box().box_h();
    //   lanemark.lanemark_box.score = out_lanemark.lanemark_box().score();
    //   lanemark.lanemark_box.type = static_cast<uint8_t>(out_lanemark.lanemark_box().type());
    //   msg->lanemark_result.lanemark_objects.push_back(lanemark);
    // }

    // // ipm mask
    // for (int i = 0; i < proto_msg.results_msk().ipm_class_size(); ++i) {
    //   caic_perception::IPMMaskClass ipm_class;

    //   auto out_ipm_class = proto_msg.results_msk().ipm_class(i);
    //   for (int j = 0; j < out_ipm_class.ipm_contour_size(); ++j) {
    //     caic_perception::IPMMaskContour ipm_contour;

    //     auto out_ipm_contour = out_ipm_class.ipm_contour(j);
    //     for (int k = 0; k < out_ipm_contour.ipm_points_size(); ++k) {
    //       auto out_pt = out_ipm_contour.ipm_points(k);

    //       caic_perception::SurroundPoint ipm_point;
    //       ipm_point.ipm_point.x = out_pt.ipm_x();
    //       ipm_point.ipm_point.y = out_pt.ipm_y();
    //       ipm_point.vcs_point.x = out_pt.vcs_x();
    //       ipm_point.vcs_point.y = out_pt.vcs_y();

    //       ipm_contour.ipm_points.push_back(ipm_point);
    //     }
    //     ipm_class.ipm_contours.push_back(ipm_contour);
    //   }
    //   msg->ipm_mask_result.ipm_classes.push_back(ipm_class);
    // }

    // // info_ipm_vcs
    // msg->info_ipm_vcs.x_diff = static_cast<uint16_t>(proto_msg.info_ipm_vcs().x_diff());
    // msg->info_ipm_vcs.y_diff = static_cast<uint16_t>(proto_msg.info_ipm_vcs().y_diff());
    // msg->info_ipm_vcs.ratio = static_cast<uint16_t>(proto_msg.info_ipm_vcs().ratio());
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    // proto_msg->Clear();
    // auto header = proto_msg->mutable_header();
    // header->set_timestamp_sec(msg.header.stamp * 1.0 / 1000000l);

    // // parking space
    // auto parking_space_frame = proto_msg->mutable_parking_space_frame();
    // for (size_t i = 0; i < msg.parking_space_result.parking_spaces.size(); ++i) {
    //   auto out_parking = msg.parking_space_result.parking_spaces[i];
    //   auto parking = parking_space_frame->add_parking_space();
    //   parking->set_timestamp(msg.header.stamp * 1.0 / 1000000l);
    //   parking->set_track_id(out_parking.track_id);
    //   parking->set_lost_age(out_parking.lost_age);
    //   parking->set_score(out_parking.score);
    //   parking->set_type(static_cast<int32_t>(out_parking.type));
    //   parking->set_occu(static_cast<int32_t>(out_parking.occu));
    //   auto space_vcs = parking->mutable_space_vcs();
    //   space_vcs->set_x1(out_parking.space_vcs.parkingspace_left_up_point.x);
    //   space_vcs->set_y1(out_parking.space_vcs.parkingspace_left_up_point.y);
    //   space_vcs->set_x2(out_parking.space_vcs.parkingspace_right_up_point.x);
    //   space_vcs->set_y2(out_parking.space_vcs.parkingspace_right_up_point.y);
    //   space_vcs->set_x3(out_parking.space_vcs.parkingspace_right_down_point.x);
    //   space_vcs->set_y3(out_parking.space_vcs.parkingspace_right_down_point.y);
    //   space_vcs->set_x4(out_parking.space_vcs.parkingspace_left_down_point.x);
    //   space_vcs->set_y4(out_parking.space_vcs.parkingspace_left_down_point.y);
    //   space_vcs->set_chock_x1(out_parking.space_vcs.chock_left_point.x);
    //   space_vcs->set_chock_y1(out_parking.space_vcs.chock_left_point.y);
    //   space_vcs->set_chock_x2(out_parking.space_vcs.chock_right_point.x);
    //   space_vcs->set_chock_y2(out_parking.space_vcs.chock_right_point.y);
    //   space_vcs->set_angle1(out_parking.space_vcs.angle_left);
    //   space_vcs->set_angle2(out_parking.space_vcs.angle_right);
    //   space_vcs->set_left_up_point_occ_statuscar(
    //       out_parking.space_vcs.left_up_point_occ_by_statuscar);
    //   space_vcs->set_left_up_point_occ_by_obstacle(
    //       out_parking.space_vcs.left_up_point_occ_by_obstacle);
    //   space_vcs->set_left_up_point_camera_source(
    //       static_cast<int32_t>(out_parking.space_vcs.left_up_point_camera_source));
    //   space_vcs->set_right_up_point_occ_statuscar(
    //       out_parking.space_vcs.right_up_point_occ_by_statuscar);
    //   space_vcs->set_right_up_point_occ_by_obstacle(
    //       out_parking.space_vcs.right_up_point_occ_by_obstacle);
    //   space_vcs->set_right_up_point_camera_source(
    //       static_cast<int32_t>(out_parking.space_vcs.right_up_point_camera_source));

    //   auto space_ipm = parking->mutable_space_ipm();
    //   space_ipm->set_x1(out_parking.space_ipm.parkingspace_left_up_point.x);
    //   space_ipm->set_y1(out_parking.space_ipm.parkingspace_left_up_point.y);
    //   space_ipm->set_x2(out_parking.space_ipm.parkingspace_right_up_point.x);
    //   space_ipm->set_y2(out_parking.space_ipm.parkingspace_right_up_point.y);
    //   space_ipm->set_x3(out_parking.space_ipm.parkingspace_right_down_point.x);
    //   space_ipm->set_y3(out_parking.space_ipm.parkingspace_right_down_point.y);
    //   space_ipm->set_x4(out_parking.space_ipm.parkingspace_left_down_point.x);
    //   space_ipm->set_y4(out_parking.space_ipm.parkingspace_left_down_point.y);
    //   space_ipm->set_chock_x1(out_parking.space_ipm.chock_left_point.x);
    //   space_ipm->set_chock_y1(out_parking.space_ipm.chock_left_point.y);
    //   space_ipm->set_chock_x2(out_parking.space_ipm.chock_right_point.x);
    //   space_ipm->set_chock_y2(out_parking.space_ipm.chock_right_point.y);
    //   space_ipm->set_angle1(out_parking.space_ipm.angle_left);
    //   space_ipm->set_angle2(out_parking.space_ipm.angle_right);
    //   space_ipm->set_left_up_point_occ_statuscar(
    //       out_parking.space_ipm.left_up_point_occ_by_statuscar);
    //   space_ipm->set_left_up_point_occ_by_obstacle(
    //       out_parking.space_ipm.left_up_point_occ_by_obstacle);
    //   space_ipm->set_left_up_point_camera_source(
    //       static_cast<int32_t>(out_parking.space_ipm.left_up_point_camera_source));
    //   space_ipm->set_right_up_point_occ_statuscar(
    //       out_parking.space_ipm.right_up_point_occ_by_statuscar);
    //   space_ipm->set_right_up_point_occ_by_obstacle(
    //       out_parking.space_ipm.right_up_point_occ_by_obstacle);
    //   space_ipm->set_right_up_point_camera_source(
    //       static_cast<int32_t>(out_parking.space_ipm.right_up_point_camera_source));
    // }

    // // freespace
    // auto free_space_objects = proto_msg->mutable_free_space_objects();
    // for (size_t i = 0; i < msg.freespace_result.free_space_points.size(); ++i) {
    //   auto out_freespace = msg.freespace_result.free_space_points[i];
    //   auto free_space = free_space_objects->add_free_space_points();
    //   free_space->set_ipm_x(out_freespace.pt.ipm_point.x);
    //   free_space->set_ipm_y(out_freespace.pt.ipm_point.y);
    //   free_space->set_vcs_x(out_freespace.pt.vcs_point.x);
    //   free_space->set_vcs_y(out_freespace.pt.vcs_point.y);

    //   if (out_freespace.type == 0) {
    //     free_space->set_type(::apollo::perception::vision::FreespacePoint::BACKGROUND);
    //   } else if (out_freespace.type == 1) {
    //     free_space->set_type(::apollo::perception::vision::FreespacePoint::CURB);
    //   } else {
    //     free_space->set_type(::apollo::perception::vision::FreespacePoint::OBSTACLE);
    //   }
    // }

    // // lanemark
    // auto lanemarks = proto_msg->mutable_result_lanemark();
    // for (size_t i = 0; i < msg.lanemark_result.lanemark_objects.size(); ++i) {
    //   auto lanemark = lanemarks->add_lanemark_objects();
    //   lanemark->set_mark_id(msg.lanemark_result.lanemark_objects[i].mark_id);

    //   auto points = msg.lanemark_result.lanemark_objects[i].lanemark_points;
    //   for (size_t j = 0; j < points.size(); ++j) {
    //     auto lanemark_points = lanemark->add_lanemark_points();
    //     lanemark_points->set_pt_ipm_x(points[j].pt.ipm_point.x);
    //     lanemark_points->set_pt_ipm_y(points[j].pt.ipm_point.y);
    //     lanemark_points->set_pt_vcs_x(points[j].pt.vcs_point.x);
    //     lanemark_points->set_pt_vcs_y(points[j].pt.vcs_point.y);
    //     lanemark_points->set_score(points[j].score);
    //     lanemark_points->set_type(static_cast<int32_t>(points[j].type));
    //   }

    //   auto box = msg.lanemark_result.lanemark_objects[i].lanemark_box;
    //   auto lanemark_box = lanemark->mutable_lanemark_box();
    //   lanemark_box->set_pt_ipm_x(box.pt.ipm_point.x);
    //   lanemark_box->set_pt_ipm_y(box.pt.ipm_point.y);
    //   lanemark_box->set_pt_vcs_x(box.pt.vcs_point.x);
    //   lanemark_box->set_pt_vcs_y(box.pt.vcs_point.y);
    //   lanemark_box->set_box_w(box.box_w);
    //   lanemark_box->set_box_h(box.box_h);
    //   lanemark_box->set_score(box.score);
    //   lanemark_box->set_type(static_cast<int32_t>(box.type));
    // }

    // // ipm mask
    // auto ipm_mask = proto_msg->mutable_results_msk();
    // for (size_t i = 0; i < msg.ipm_mask_result.ipm_classes.size(); ++i) {
    //   auto ipm_class = ipm_mask->add_ipm_class();

    //   auto out_class = msg.ipm_mask_result.ipm_classes[i];
    //   for (size_t j = 0; j < out_class.ipm_contours.size(); ++j) {
    //     auto ipm_contour = ipm_class->add_ipm_contour();

    //     auto out_contour = out_class.ipm_contours[j];
    //     for (size_t k = 0; k < out_contour.ipm_points.size(); ++k) {
    //       auto ipm_points = ipm_contour->add_ipm_points();
    //       ipm_points->set_ipm_x(out_contour.ipm_points[k].ipm_point.x);
    //       ipm_points->set_ipm_y(out_contour.ipm_points[k].ipm_point.y);
    //       ipm_points->set_vcs_x(out_contour.ipm_points[k].vcs_point.x);
    //       ipm_points->set_vcs_y(out_contour.ipm_points[k].vcs_point.y);
    //     }
    //   }
    // }

    // // info_ipm_vcs
    // auto info_ipm_vcs = proto_msg->mutable_info_ipm_vcs();
    // info_ipm_vcs->set_x_diff(static_cast<int32_t>(msg.info_ipm_vcs.x_diff));
    // info_ipm_vcs->set_y_diff(static_cast<int32_t>(msg.info_ipm_vcs.y_diff));
    // info_ipm_vcs->set_ratio(static_cast<int32_t>(msg.info_ipm_vcs.ratio));
  }
};

template <>
struct AdaptorTraits<caic_sensor::Canbus, true> {
  using Type = caic_sensor::Canbus;
  using ProtoType = ::apollo::drivers::Chassis;
  using ROSType = std_msgs::String;

  static void convert(const ROSType& ros_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ros_msg.data);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, ROSType* ros_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ros_msg->data);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    msg->header.stamp = proto_msg.header().timestamp_sec() * 1000000.0;
    msg->driving_mode = (caic_sensor::DrivingMode)proto_msg.driving_mode();
    msg->chassis_info.esp.vehicle_speed = proto_msg.speed_mps();
    msg->chassis_info.esp.wheel_pulse_info.fl = proto_msg.wheel_pluse().wheel_pluse_fl();
    msg->chassis_info.esp.wheel_pulse_info.fr = proto_msg.wheel_pluse().wheel_pluse_fr();
    msg->chassis_info.esp.wheel_pulse_info.rl = proto_msg.wheel_pluse().wheel_pluse_rl();
    msg->chassis_info.esp.wheel_pulse_info.rr = proto_msg.wheel_pluse().wheel_pluse_rr();
    msg->chassis_info.eps.steering_wheel_info.angle =
        proto_msg.steering_percentage() * 8.726646 / 100;
    msg->chassis_info.eps.steering_wheel_info.speed = proto_msg.steeringwheelspeed();
    msg->chassis_info.eps.steering_wheel_info.speed_sign = proto_msg.steeringwheelspeedsign();
    msg->chassis_info.esp.wheel_speed_info.fl = proto_msg.wheel_speed().wheel_spd_fl();
    msg->chassis_info.esp.wheel_speed_info.fr = proto_msg.wheel_speed().wheel_spd_fr();
    msg->chassis_info.esp.wheel_speed_info.rl = proto_msg.wheel_speed().wheel_spd_rl();
    msg->chassis_info.esp.wheel_speed_info.rr = proto_msg.wheel_speed().wheel_spd_rr();
    switch (proto_msg.wheel_speed().wheel_direction_rr()) {
      case 0:
        msg->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::FORWARD;
        break;
      case 1:
        msg->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::BACKWARD;
        break;
      case 2:
        msg->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::STOP;
        break;
      case 3:
        msg->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::INVALID_VALUE;
        break;
      default:
        break;
    }

    switch (proto_msg.gear_location()) {
      case 0:
        msg->chassis_info.gear_info = caic_std::GearState::NEUTRAL;
        break;
      case 1:
        msg->chassis_info.gear_info = caic_std::GearState::DRIVE;
        break;
      case 2:
        msg->chassis_info.gear_info = caic_std::GearState::REVERSE;
        break;
      case 3:
        msg->chassis_info.gear_info = caic_std::GearState::PARKING;
        break;
      default:
        msg->chassis_info.gear_info = caic_std::GearState::INVALID;
        break;
    }
  }
  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    // 1.0 head
    // proto::common::Header* header = proto_msg->mutable_header();
    // // header->set_seq(msg.header.seq);
    // header->set_stamp(msg.header.stamp);

    // proto_msg->mutable_chassis_info()->mutable_esp()->set_vehicle_speed(
    //     msg.chassis_info.esp.vehicle_speed);
    // proto_msg->mutable_chassis_info()->set_gear_info(
    //     (proto::sensor::Chassis_Info_Gear_Info)(msg.chassis_info.gear_info));
    // proto_msg->mutable_chassis_info()->mutable_yrs()->set_yaw_rate(msg.chassis_info.yrs.yaw_rate);
    // proto_msg->mutable_chassis_info()->mutable_esp()->mutable_wheel_speed_info()->set_fl(
    //     msg.chassis_info.esp.wheel_speed_info.fl);
    // proto_msg->mutable_chassis_info()->mutable_esp()->mutable_wheel_speed_info()->set_fr(
    //     msg.chassis_info.esp.wheel_speed_info.fr);
    // proto_msg->mutable_chassis_info()->mutable_esp()->mutable_wheel_speed_info()->set_rl(
    //     msg.chassis_info.esp.wheel_speed_info.rl);
    // proto_msg->mutable_chassis_info()->mutable_esp()->mutable_wheel_speed_info()->set_rr(
    //     msg.chassis_info.esp.wheel_speed_info.rr);
    // TODO set frame_id
    // 2.0 meta
    // proto::common::OdomMeta* meta = rt_msg.mutable_meta();
    // meta->set_start_timestamp_us(msg.meta.start_timestamp_us);
    // meta->set_finish_timestamp_us(msg.meta.finish_timestamp_us);
    // 3.0
    // rt_msg.set_available((uint64_t)msg.available);
    // 4.0
    // rt_msg.set_child_frame_id(msg.child_frame_id);
    // 5.0
    // 6.0
    // 7.0
    printf("convert odo, pub msg: %d\n",
           ((proto::sensor::Chassis_Info_Gear_Info)(msg.chassis_info.gear_info)));
  }
};

// template <>
// struct AdaptorTraits<msgs::sensor::GNSS, true> {
//   using Type = msgs::sensor::GNSS;
//   using ROSType = nav_msgs::Odometry;

//   static void convert(const ROSType& ros_msg, Type* msg) {
//     msg->header.seq = ros_msg.header.seq;
//     msg->header.stamp = ros_msg.header.stamp.sec * 1000000.0 + ros_msg.header.stamp.nsec /
//     1000.0; msg->header.frame_id = ros_msg.header.frame_id; msg->pose.position.x =
//     ros_msg.pose.pose.position.x; msg->pose.position.y = ros_msg.pose.pose.position.y;
//     msg->pose.position.z = ros_msg.pose.pose.position.z;
//     msg->pose.orientation.x = ros_msg.pose.pose.orientation.x;
//     msg->pose.orientation.y = ros_msg.pose.pose.orientation.y;
//     msg->pose.orientation.z = ros_msg.pose.pose.orientation.z;
//     msg->pose.orientation.w = ros_msg.pose.pose.orientation.w;
//     for (size_t i = 0; i < 36; ++i) {
//       msg->pose_cov(i / 6, i % 6) = ros_msg.pose.covariance[i];
//     }
//     msg->twist.linear.x = ros_msg.twist.twist.linear.x;
//     msg->twist.linear.y = ros_msg.twist.twist.linear.y;
//     msg->twist.linear.z = ros_msg.twist.twist.linear.z;
//     msg->twist.angular.x = ros_msg.twist.twist.angular.x;
//     msg->twist.angular.y = ros_msg.twist.twist.angular.y;
//     msg->twist.angular.z = ros_msg.twist.twist.angular.z;
//     for (size_t i = 0; i < 36; ++i) {
//       msg->twist_cov(i / 6, i % 6) = ros_msg.twist.covariance[i];
//     }
//   }

//   static void convert(const Type&, ROSType*) {
//     std::cout << "GNSS deserialize not implemented." << std::endl;
//   }
// };

// template <std::size_t N>
// struct AdaptorTraits<msgs::sensor::PointCloudBase<msgs::geometry::PointXYZIRTLf, N>, true> {
//   using Type = msgs::sensor::PointCloudBase<msgs::geometry::PointXYZIRTLf, N>;
//   using ROSType = sensor_msgs::PointCloud2;

//   static void convert(const ROSType& ros_msg, Type* msg) {
//     msg->header.seq = ros_msg.header.seq;
//     msg->header.stamp = ros_msg.header.stamp.sec * 1000000.0 + ros_msg.header.stamp.nsec /
//     1000.0;
//     // double timestamp_base;
//     // std::memcpy(&timestamp_base, &ros_msg.data[24], 8);
//     // msg->header.stamp = timestamp_base * 1000000.0;
//     msg->header.frame_id = ros_msg.header.frame_id;
//     msg->size = ros_msg.width * ros_msg.height;
//     int32_t type = 0;
//     if (ros_msg.fields[4].name == "ring") {
//       type = 1;
//     } else if (ros_msg.fields[4].name == "timestamp") {
//       type = 2;
//     }
//     for (size_t i = 0; i < msg->size; i++) {
//       uint8_t intensity = 0;
//       double timestamp;
//       Eigen::Map<Eigen::VectorXf> xyz_in(reinterpret_cast<float*>(const_cast<unsigned char*>(
//                                              &ros_msg.data[ros_msg.point_step * i])),
//                                          16);
//       Eigen::Map<Eigen::VectorXf> xyz_out(reinterpret_cast<float*>(&msg->points[i]), 16);
//       xyz_out = xyz_in;
//       msg->points[i].intensity = (uint16_t)intensity;
//       switch (type) {
//         case 1: {
//           std::memcpy(&msg->points[i].ring, &ros_msg.data[ros_msg.point_step * i + 18], 2);
//           break;
//         }
//         case 2: {
//           std::memcpy(&msg->points[i].ring, &ros_msg.data[ros_msg.point_step * i + 32], 2);
//           break;
//         }
//         default:
//           break;
//       }
//       std::memcpy(&timestamp, &ros_msg.data[ros_msg.point_step * i + 24], 8);
//       msg->points[i].timestamp_2us = (msg->header.stamp - timestamp * 1000000.0) / 2;
//     }
//   }

//   static void convert(const Type&, ROSType*) {
//     std::cout << "PointCloud deserialize not implemented." << std::endl;
//   }
// };

// PRQA S 4400,4402,6006 ++ # Todo: to be solved
template <>
struct AdaptorTraits<::caic_sensor::PointCloudTypeArray, true> {
  using Type = ::caic_sensor::PointCloudTypeArray;
  using ROSType = sensor_msgs::PointCloud2;

  static void convert(const ROSType& ros_msg, Type* const msg) {
    // msg->header.seq = ros_msg.header.seq;
    // msg->header.frame_id = ros_msg.header.frame_id;
    int data_length = 16;  // PRQA S 4400
    msg->timestamp = ros_msg.header.stamp.sec * 1000000.0 + ros_msg.header.stamp.nsec / 1000.0;
    msg->width = ros_msg.width;
    msg->height = ros_msg.height;
    msg->size = ros_msg.width * ros_msg.height;
    int32_t type = 0;
    if (ros_msg.fields[3].name == "intensity") {
      type = 1;
    } else if (ros_msg.fields[4].name == "ring") {
      type = 2;
    }

    for (size_t i = 0; i < msg->size; i++) {
      Eigen::Map<Eigen::VectorXf> xyz_in(reinterpret_cast<float*>(const_cast<unsigned char*>(
                                             &ros_msg.data[ros_msg.point_step * i])),
                                         data_length);
      Eigen::Map<Eigen::VectorXf> xyz_out(reinterpret_cast<float*>(&msg->points[i]), data_length);
      xyz_out = xyz_in;
      switch (type) {
        case 1: {
          std::memcpy(&msg->points[i].intensity, &ros_msg.data[ros_msg.point_step * i + 16], 2);
          break;
        }
        case 2: {
          std::memcpy(&msg->points[i].ring, &ros_msg.data[ros_msg.point_step * i + 18], 4);
          break;
        }
        default:
          break;
      }
      double timestamp;
      std::memcpy(&timestamp, &ros_msg.data[ros_msg.point_step * i + 24], 8);
      msg->points[i].timestamp_2us = (msg->timestamp - timestamp * 1000000.0) / 2;
    }
  }

  static void convert(const Type&, ROSType*) {
    std::cout << "PointCloud deserialize not implemented." << std::endl;
  }
};
// PRQA S 4400,4402,6006 --

template <>
struct AdaptorTraits<::caic_sensor::Image, true> {
  using Type = ::caic_sensor::Image;
  using ROSType = sensor_msgs::Image;

  static void convert(const ROSType& ros_msg, Type* const msg) {
    msg->header.seq = ros_msg.header.seq;
    msg->header.stamp = ros_msg.header.stamp.sec * 1000000.0 + ros_msg.header.stamp.nsec / 1000.0;
    // msg->header.frame_id = ros_msg.header.frame_id;
    msg->width = ros_msg.width;
    msg->height = ros_msg.height;
    msg->size = ros_msg.width * ros_msg.height * 3;  // PRQA S 4400 # dimension is 3
    // msg->is_bigendian = ros_msg.is_bigendian;
    msg->p_data = (uint8_t*)(&ros_msg.data[0]);
  }

  static void convert(const Type&, ROSType*) {
    std::cout << "3840x2160 deserialize not implemented." << std::endl;
  }
};

template <typename _T>
struct AdaptorTraits<_T, false> {
  using Type = _T;
  using ROSType = std_msgs::String;

  static void convert(const ROSType& ros_msg, Type* msg) { msg->ParseFromString(ros_msg.data); }

  static void convert(const Type& msg, ROSType* ros_msg) { msg.SerializeToString(&ros_msg->data); }
};

template <typename _T>
struct AdaptorTraits<_T, true> {
  using Type = _T;
  using ROSType = std_msgs::String;

  static void convert(const ROSType&, Type*) {}

  static void convert(const Type&, ROSType*) {}
};

template <typename _Msg, typename _ROSMsg, bool _Adaptive>
void callbackAdaptor(const typename _ROSMsg::ConstPtr& ros_msg, void (*callback)(const _Msg&)) {
  _Msg msg;
  AdaptorTraits<_Msg, _Adaptive>::convert(*ros_msg, &msg);
  callback(msg);
}

template <typename _Msg, typename _ROSMsg, bool _Adaptive>
void callbackAdaptorBoost(typename _ROSMsg::ConstPtr ros_msg,
                          const boost::function<void(std::shared_ptr<_Msg>)>& callback) {
  static stoic::cm::alg::ObjectPool<_Msg> pool(9);
  std::shared_ptr<_Msg> msg = pool.calloc();
  AdaptorTraits<_Msg, _Adaptive>::convert(*ros_msg, msg.get());
  callback(msg);
}

}  // namespace stoic::cm
