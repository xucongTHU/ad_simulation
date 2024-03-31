// Copyright 2022 The XUCONG Authors. All Rights Reserved.

#include "bridge_gflags.h"

DEFINE_string(bridge_module_name, "Bridge", "Bridge module name");
DEFINE_string(udp_bridge_conf_file, "/home/autopilot/stoic/src/app/configs/bridge/config.yaml", "birdge config file");
DEFINE_int64(min_cmd_interval, 5, "Minimum control command interval in ms.");
