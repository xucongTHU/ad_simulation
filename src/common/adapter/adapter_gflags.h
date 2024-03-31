// Copyright 2022 The XUCONG Authors. All Rights Reserved.

#ifndef ADAPTER_GFLAGS_H
#define ADAPTER_GFLAGS_H

#include "gflags/gflags.h"


// TO DO
DECLARE_string(pointcloud_topic);
DECLARE_string(radar_topic);
DECLARE_string(ultrasonic_radar_topic);
DECLARE_string(image_topic);


// New architecture
DECLARE_string(sim_imu_topic);
DECLARE_string(sim_gps_topic);
DECLARE_string(sim_perception_objects_topic);
DECLARE_string(sim_perception_traffic_light_topic);
DECLARE_string(sim_actor_control_topic);
DECLARE_string(sim_chassis_topic);
DECLARE_string(sim_localization_topic);
DECLARE_string(sim_lane_topic);
DECLARE_string(chassis_topic);
DECLARE_string(localization_topic);
DECLARE_string(control_topic);
DECLARE_string(perception_topic);
DECLARE_string(traffic_light_topic);

#endif // ADAPTER_GFLAGS_H
