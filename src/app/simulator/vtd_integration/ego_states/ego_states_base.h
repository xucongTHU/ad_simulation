//
// Created by xucong on 24-1-9.
//

#ifndef SIMULATOR_VTD_INTEGRATION_EGO_STATES_EGO_STATES_BASE_H_
#define SIMULATOR_VTD_INTEGRATION_EGO_STATES_EGO_STATES_BASE_H_

#include "simulator/vtd_integration/vtd_dependencies/common/viRDBIcd.h"

typedef struct {
  RDB_MSG_HDR_t hdr;
  RDB_MSG_ENTRY_HDR_t entrySOF;
  RDB_MSG_ENTRY_HDR_t entryTrigger;
  RDB_TRIGGER_t rdbTrigger;
  RDB_MSG_ENTRY_HDR_t entryEgo;
  RDB_OBJECT_STATE_t egoState;
  RDB_MSG_ENTRY_HDR_t entryWheel;
  RDB_WHEEL_t wheels[4];
  RDB_MSG_ENTRY_HDR_t entryEOF;
} RDB_EGO_STATE_t;

#endif //SIMULATOR_VTD_INTEGRATION_EGO_STATES_EGO_STATES_BASE_H_
