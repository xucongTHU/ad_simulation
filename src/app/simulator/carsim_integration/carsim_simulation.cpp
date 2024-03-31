// Copyright 2023 The XUCONG Authors. All Rights Reserved.

#if (defined(_WIN32) || defined(_WIN64))
#include <windows.h> // Windows-specific header
#endif

#include <stdio.h>
#include <string>

#include "src/app/simulator/carsim_integration/vehicle_states_calc/vehicle_states_calc.h"

using namespace stoic::simulator;
stoic::UdpSocketClient<sim_chassis::EgoStates> websocket_ego;

void send_ego_vehicle_info(const double &simTime, const int &simFrame) {
  using egoType = sim_chassis::EgoStates;
  egoType ego_vehicle;

  websocket_ego.connectToUdpServer(ego_vehicle_ip, ego_vehicle_port);

  ego_vehicle.simFrame                             = simFrame;
  ego_vehicle.simTime                              = simTime;
  ego_vehicle.available                            = 1;
  ego_vehicle.pose.pose_info.x                     = *(vs_out.pos.x);
  ego_vehicle.pose.pose_info.y                     = *(vs_out.pos.y);
  ego_vehicle.pose.pose_info.z                     = *(vs_out.pos.z);
  ego_vehicle.pose.linear_velocity_info.x          = *(vs_out.velocity.x);
  ego_vehicle.pose.linear_velocity_info.y          = *(vs_out.velocity.y) / 3.6;
  ego_vehicle.pose.linear_velocity_info.z          = *(vs_out.velocity.z);
  ego_vehicle.pose.linear_acceleration_vrf_info.x  = *(vs_out.accel.x);
  ego_vehicle.pose.linear_acceleration_vrf_info.y  = *(vs_out.accel.y);
  ego_vehicle.pose.linear_acceleration_vrf_info.z  = *(vs_out.accel.z);
  ego_vehicle.pose.angular_velocity_vrf_info.x     = float(*(vs_out.angular_vel.x));
  ego_vehicle.pose.angular_velocity_vrf_info.y     = float(*(vs_out.angular_vel.y));
  ego_vehicle.pose.angular_velocity_vrf_info.z     = float(*(vs_out.angular_vel.z));
  ego_vehicle.pose.angle.roll                      = float(*(vs_out.roll));
  ego_vehicle.pose.angle.pitch                     = float(*(vs_out.pitch));
  ego_vehicle.pose.angle.yaw                       = float(*(vs_out.yaw));
  ego_vehicle.pose.angular_acceleration_vrf_info.x = float(*(vs_out.angular_acc.x));
  ego_vehicle.pose.angular_acceleration_vrf_info.y = float(*(vs_out.angular_acc.y));
  ego_vehicle.pose.angular_acceleration_vrf_info.z = float(*(vs_out.angular_acc.z));
  ego_vehicle.tire_info.effective_radius_info.fl   = float(*(vs_out.RRE_L1));
  ego_vehicle.tire_info.effective_radius_info.fr   = float(*(vs_out.RRE_L2));
  ego_vehicle.tire_info.effective_radius_info.rl   = float(*(vs_out.RRE_R1));
  ego_vehicle.tire_info.effective_radius_info.rr   = float(*(vs_out.RRE_R2));
  ego_vehicle.wheel_info.wheel_rotation_info.fl    = float(*(vs_out.Rot_L1));
  ego_vehicle.wheel_info.wheel_rotation_info.fr    = float(*(vs_out.Rot_L2));
  ego_vehicle.wheel_info.wheel_rotation_info.rl    = float(*(vs_out.Rot_R1));
  ego_vehicle.wheel_info.wheel_rotation_info.rr    = float(*(vs_out.Rot_R2));
  ego_vehicle.steer_info.steering_wheel_info.fl    = float(*(vs_out.Steer_L1));
  ego_vehicle.steer_info.steering_wheel_info.fr    = float(*(vs_out.Steer_L2));
  ego_vehicle.steer_info.steering_wheel_info.rl    = float(*(vs_out.Steer_R1));
  ego_vehicle.steer_info.steering_wheel_info.rr    = float(*(vs_out.Steer_R2));
  ego_vehicle.wheel_info.wheel_spin_info.fl        = float(*(vs_out.Avy_L1));
  ego_vehicle.wheel_info.wheel_spin_info.fr        = float(*(vs_out.Avy_L2));
  ego_vehicle.wheel_info.wheel_spin_info.rl        = float(*(vs_out.Avy_R1));
  ego_vehicle.wheel_info.wheel_spin_info.rr        = float(*(vs_out.Avy_R2));

  websocket_ego.sendData(ego_vehicle);
  websocket_ego.closeSocket();
}

/* ---------------------------------------------------------------------------------
   Main program to run DLL with VS API.
--------------------------------------------------------------------------------- */
int main(int argc, char **argv) {
  HMODULE vsDLL = NULL; // DLL with VS API
  char pathDLL[FILENAME_MAX], simfile[FILENAME_MAX] = {};

  double t = 0, tStep = 0.02; // stoic time
  // init pos and heading
  vs_init.initX   = (double*) malloc(sizeof(double));
  vs_init.initY   = (double*) malloc(sizeof(double));
  vs_init.initZ   = (double*) malloc(sizeof(double));
  vs_init.initYaw = (double*) malloc(sizeof(double));
  vs_init.initV   = (double*) malloc(sizeof(double));

  *(vs_init.initX) = -4417.73;
  *(vs_init.initY) = -13110.34;
  *(vs_init.initZ) = 0;
  *(vs_init.initYaw) = 3.109;
  *(vs_init.initV) = 11.11;

  YAML::Node config;
  config = YAML::LoadFile(FLAGS_carsim_config);
  std::string simfile_path = config["sil"]["simfile_dir"].as<std::string>();
  double simTime = config["sil"]["simtime"].as<double>();
  strcpy(simfile, simfile_path.c_str());
  // get simfile from argument list and load DLL  
//  if (argc > 1) strcpy (simfile, &argv[1][0]);
  if (vs_get_dll_path(simfile, pathDLL)) 
    return 1;
#if (defined(_WIN32) || defined(_WIN64))
  vsDLL = LoadLibrary(pathDLL);
#else
  vsDLL = vs_load_library(pathDLL);
#endif
  // get API functions
  if (vs_get_api(vsDLL, pathDLL)) 
    return 1;

  // install external functions from custom code
  vs_install_setdef_function(external_setdef);
  vs_install_calc_function(external_calc);
  vs_install_echo_function(external_echo);
  vs_install_scan_function(external_scan);

  t = vs_setdef_and_read(simfile, NULL, NULL);
  if (vs_error_occurred()){
    printf("\n\nError occurred reading simfile \"%s\"", simfile);
    return 1;
  }

  vs_initialize(t, NULL, NULL);
  printf("stoic initialized!\n");

  int simFrameNo = 0;
  while (t < simTime) {
    for (int i = 0; i < 40; i++) {
      vs_integrate(&t, NULL);
    }

    fprintf(stderr, "tStep = %.2f\n", t);
    fprintf(stderr, "position = %.2lf / %.2lf / %.2lf\n", *(vs_out.pos.x), *(vs_out.pos.y), *(vs_out.pos.z));

    send_ego_vehicle_info(t, simFrameNo);

    simFrameNo++;

  }

  // Terminate
  vs_terminate_run(t);
  printf("stoic finished\n");
  // pause?
  if (vs_opt_pause())
  {
    printf("\nOPT_PAUSE was set to keep this display visible."
           "\nPress the Return key to exit. ");
    fgetc(stdin);
  }
  vs_free_library(vsDLL);

  return 0;
}
