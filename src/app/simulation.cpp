
//
// Created by xucong on 24-1-10.
//

#include "common/log/Logger.h"
#include "app/workflow/workflow_ART.hpp"
#ifdef MW_ROS_IPC || MW_ROS_ORIN
#include "app/workflow/workflow_ROS.hpp"
#endif
static const char* LOG_TAG = "main";
using namespace stoic;
using namespace stoic::app;
using namespace stoic::pattern;
using namespace stoic;
void sighandler(int signum) {
  printf("捕获到退出信号: %d. 程序即将退出.\n", signum);
  exit(-1);
}
int main (int argc, char** argv) {
  if (argc <= 1) {
    std::cerr << "Usage: ./sim Deployment" << std::endl;;
    std::cerr << "    [PERC] short for perc module" << std::endl;
    std::cerr << "    [DRI] short for driver module" << std::endl;
    std::cerr << "    [CAN] short for canbus module" << std::endl;
    std::cerr << "    [LOC] short for localization module" << std::endl;
    std::cerr << "    [ALL] short for all module" << std::endl;
    exit(-1);
  }
  std::string log_dir_prefix;
  std::string platform;
#if defined MW_ROS_IPC
  platform = "x86/ros";
  log_dir_prefix = "/home/autopilot/stoic/log/" + platform + "/log/";
#elif defined MW_ROS_ORIN
  platform = "arm/ros";
  log_dir_prefix = "/home/autopilot/stoic/log/" + platform + "/log/";
#elif defined MW_AP
  platform = "arm/ap";
  log_dir_prefix = "/home/autopilot/stoic/log/" + platform + "/log/";
#elif defined MW_ART_IPC || defined MW_ART_ORIN
  platform = "x86/ART";
  log_dir_prefix = "/home/autopilot/stoic/log/" + platform + "/log/";
#endif
  
  std::string deployment = argv[1];
  caic_logger::Logger::instance()->Init(caic_logger::LOG_TO_CONSOLE | caic_logger::LOG_TO_FILE,
                                        LOG_LEVEL_INFO, (log_dir_prefix + deployment + "_log.txt").c_str(), nullptr);
  stoic::cm::init(argc, argv, "stoic_" + deployment);
  stoic::cm::NodeHandle nh("stoic_" + deployment);
  signal(SIGINT, sighandler);
  Workflow wf;
  WorkflowLoader wfl(nh, deployment);
  wfl.init(&wf);
  stoic::cm::spin();
  caic_logger::Logger::instance()->Uninit();
  return 0;
}
