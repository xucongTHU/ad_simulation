//
// Created by xucong on 23-8-30.
//


#include "dirent.h"
#include "common/log/Logger.h"
#include "app/simulator/vtd_integration/workflow_perc.hpp"

static const char* LOG_TAG = "main";
using namespace stoic::simulator;
int main (int argc, char** argv) {
  if (argc <= 1) {
    std::cerr << "Usage: ./vtd_stoic Deployment" << std::endl;;
    std::cerr << "    [FVIS] short for front vision module" << std::endl;
    std::cerr << "    [BFW] short for bev fr camera module" << std::endl;
    std::cerr << "    [ALL] short for all module" << std::endl;
    exit(-1);
  }

  std::string log_dir_prefix = "./log";

  DIR *dir = opendir(log_dir_prefix.c_str());
  if (dir == NULL) {
    std::string command = "mkdir -p " + log_dir_prefix;
    int ret = system(command.c_str());
    if (ret != 0) {
      std::cout << "make direction for log error!!" << std::endl;;
      exit(-1);
    }
  }

  std::string log_file = "log.txt";
  log_file = log_dir_prefix + std::string("/") + log_file;

  caic_logger::Logger::instance()->Init(caic_logger::LOG_TO_CONSOLE | caic_logger::LOG_TO_FILE,
                                        LOG_LEVEL_INFO, (log_file).c_str(), nullptr);


  std::string deployment = argv[1];

  simulator::pattern::Workflow wf;
  WorkflowLoader wfl(deployment);

  while (true) {
    wfl.init(&wf);
  }

  caic_logger::Logger::instance()->Uninit();

  return 0;
}
