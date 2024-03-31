//
// Created by xucong on 23-12-20.
//

#ifndef SIM_INTERFACE_SIM_POD_CFG_H_
#define SIM_INTERFACE_SIM_POD_CFG_H_

//STD
#define STD_RESERVED_INFO_SIZE 512
#define STD_FRAME_ID_SIZE 32
#define STD_MODULE_NAME_SIZE 32
#define STD_LANE_ID_SIZE 32

//control cmd
///none

//sim msf loc
///none

//sim chassis
///none

//sim fusion
///BigFusion->FusionObjectsTab: fusion_object_tab
#define FU_OBJ_TAB_SIZE 256
///BigFusion->FusionLanes: lanes
#define FU_LANES_SIZE 16
//BigFusion->FusionLanes->FusionLane: poly_array
#define FU_LANE_CUBICPOLY_SIZE 10
///BigFusion->FusionTrafficLights: trafficlights
#define FU_TRAFFICLIGHTS_SIZE 32
///BigFusion->FusionTrafficSign: traffic_signs
#define FU_TRAFFIC_SIGNS_SIZE 32
///BigFusion->FusionParkSpace: parking_space
#define FU_PARKING_SPACE_SIZE 64
///BigFusion->FusionFreeSpace: freespace_points
#define FU_FS_POINTS_SIZE 1024
///BigFusion->FusionParkSpace->ParkingSpace: psd_id
#define FU_PSD_ID_SIZE 64

//sim perception
///PerceptionLanes: lanes
#define PERC_LANES_SIZE 16
///PerceptionLanes->PerceptionLane: points
#define PERC_LANE_PTS_SIZE 128
///PerceptionLanes: stop_lines
#define PERC_STOPLINE_SIZE 8
///PerceptionLanes->PerceptionStopLine: points
#define PERC_STOPLINE_PTS_SIZE 32
///PerceptionBEVLaneBounds: lanes
#define PERC_BEV_LANES_SIZE 50
///PerceptionBEVLaneBounds->BEVLaneBound: points
#define PERC_BEV_LB_PTS_SIZE 20
///PerceptionSurround->ParkingSpaceResult: parking_spaces
#define PS_SIZE 20
///PerceptionSurround->FreespaceResult: free_space_points
#define FS_PTS_SIZE 360
///PerceptionSurround->LanemarkResult: lanemark_objects
#define PERC_LANEMARK_OBJ_SIZE 10
///PerceptionSurround->LanemarkResult->LanemarkObject: lanemark_points
#define PERC_LANEMARK_OBJ_PTS_SIZE 20
///PerceptionSurround->IPMMaskResult: ipm_classes
#define SEG_CLASS 15
///PerceptionSurround->IPMMaskResult->IPMMaskClass: ipm_contours
#define SEG_CONTOURS 100
///PerceptionSurround->IPMMaskResult->IPMMaskClass->IPMMaskContour: ipm_points
#define SEG_CT_PTS 5000
///PerceptionSurround->PerceptionObjects: objects
#define OBJ_SIZE 100
///PerceptionSurround->PerceptionObjects->PerceptionObject: fea_vec
#define FEA_VEC_SIZE 256
///PerceptionTrafficLights: traffic_lights
#define PERC_TL_SIZE 30
///PerceptionTrafficLights->TrafficLight: id
#define PERC_TL_ID_SIZE 64
///PerceptionFreespacePoints: freespace_points
#define PERC_FS_PTS_SIZE 360

#endif //SIM_INTERFACE_SIM_POD_CFG_H_
