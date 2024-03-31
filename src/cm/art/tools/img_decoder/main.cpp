// Copyright 2022 The T3CAIC.COM Authors. All Rights Reserved.
#include "cm/cm.h"
#include "pattern/workflow.hpp"
#include "workflow.hpp"

using namespace ::stoic;
extern caic_sensor::ImageEncoding g_type;
extern std::string ch_in;
extern std::string ch_out;

int main(int argc, char** argv) {
  if (argc != 7) {
    printf("Help: ./app_name -t RGB/YUYV -ch_in channel_in_name -ch_out channel_out_name.\n");
    return 0;
  }
  std::string codec_type = argv[2];
  ch_in = argv[4];
  ch_out = argv[6];
  std::string node_name = ch_in + "decoder" + std::to_string(getpid());
  if (codec_type == std::string("RGB")) {
    printf("main codec_type is RGB.\n");
    g_type = caic_sensor::ImageEncoding::RGB;
  } else if (codec_type == std::string("YUYV")) {
    printf("main codec_type is YUYV.\n");
    g_type = caic_sensor::ImageEncoding::YUV422_YUYV;
  } else {
    printf("codec format is error!!!\n");
    return -1;
  }
  cm::init(argc, argv, node_name);
  cm::NodeHandle nh(node_name);
  stoic::cm::pattern::Workflow wf;
  WorkflowLoader wfl(nh, "ALL", ch_in);
  wfl.init(&wf);
  cm::spin();
  return 0;
}
