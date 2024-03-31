// Copyright 2022 The XUCONG Authors. All Rights Reserved.

#include "common/adapters/adapter_gflags.h"

// New architecture
DEFINE_string(sim_imu_topic, "/stoic/imu", "topic name");
DEFINE_string(sim_gps_topic, "/stoic/gps", "topic name");
DEFINE_string(sim_perception_objects_topic, "/stoic/perception/objects", "topic name");
DEFINE_string(sim_perception_traffic_light_topic, "/stoic/perception/traffic_light", "topic name");
DEFINE_string(sim_actor_control_topic, "/stoic/actor/control", "topic name");
DEFINE_string(sim_chassis_topic, "/stoic/chassis", "chassis topic name");
DEFINE_string(sim_localization_topic, "/stoic/localization","localization topic name");
DEFINE_string(sim_lane_topic, "/stoic/lane_line", "lane line topic name");
DEFINE_string(chassis_topic, "/smartcar/canbus/chassis", "adas chassis topic name");
DEFINE_string(localization_topic, "/smartcar/localization/pose", "adas localization topic name");
DEFINE_string(control_topic, "/ctl_pub", "adas control command topic name");
DEFINE_string(perception_topic, "/bfus_pub", "object fusion topic name");
DEFINE_string(traffic_light_topic, "/smartcar/perception/traffic_light", "traffic light detection topic name");
