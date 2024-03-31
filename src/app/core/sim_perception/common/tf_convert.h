
//
// Created by xucong on 24-1-3.
//
#ifndef SIM_PERCEPTION_COMMON_H_
#define SIM_PERCEPTION_COMMON_H_

#include <iostream>
#include <vector>
#include <manif/manif.h>

const manif::SE3d TX_I_V(0,0,0,0,0,-M_PI_2);
const manif::SE3d TX_V_I(0,0,0,0,0,M_PI_2);
// vcs to map
void V2MTransform(const manif::SE3d& egoPos,manif::SE3d& TX_V_M)
{
  TX_V_M = egoPos*TX_V_I;
}
// map to vcs
void M2VTransform(const manif::SE3d& egoPos,manif::SE3d& TX_M_V)
{
  manif::SE3d TX_M_I = egoPos.inverse();
  TX_M_V = TX_I_V*TX_M_I;
}
#endif //SIM_PERCEPTION_COMMON_H_
