#ifndef ad_POD_CFG_H
#define ad_POD_CFG_H

///STD
#define STD_RESERVED_INFO_SIZE 512
#define STD_FRAME_ID_SIZE 32
#define STD_MODULE_NAME_SIZE 32
#define STD_LANE_ID_SIZE 32
///ad_control
///none

/// fusion
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
///FusionMap info
///BigFusion->FusionMap->FusionMapRampJunction: ramp_junctions
#define FU_RAMP_JUNCTIONS_SIZE 10
///BigFusion->FusionMap->FusionMapStopline: stop_lines
#define FU_STOPLINES_SIZE 10
///BigFusion->FusionMap->FusionMapStopline->Stopline: shape
#define FU_STOPLINE_SHAPE_PTS_SIZE 16
///BigFusion->FusionMap->FusionMapStopline->Stopline: co_lane_id
#define FU_STOPLINE_COLANE_SIZE 5
///BigFusion->FusionMap->FusionMapTrafficLight: traffic_lights
#define FU_TLIGHTS_SIZE 32
///BigFusion->FusionMap->FusionMapTrafficLight->TrafficLight: bounding_box
#define FU_TLIGHT_BBOX_PTS_SIZE 16
///BigFusion->FusionMap->FusionMapTrafficLight->TrafficLight: co_lane_id
#define FU_TLIGHT_COLANE_SIZE 5
///BigFusion->FusionMap->FusionMapTransition: transitions
#define FU_TRANSITIONS_SIZE 10
///BigFusion->FusionMap->FusionMapCrosswalk: crosswalk
#define FU_CROSSWALK_SIZE 10
///BigFusion->FusionMap->FusionMapCrosswalk->Crosswalk: shape
#define FU_CROSSWALK_SHAPE_SIZE 16
///BigFusion->FusionMap->FusionMapWaitArea: wait_area
#define FU_WAIT_AREA_SIZE 10
///BigFusion->FusionMap->FusionMapLaneInfoSection: laneinfo_section
#define FU_LANEINFO_SECTION_SIZE 10
///BigFusion->FusionMap->FusionMapLaneInfoSection->LaneInfoSection: lane_info
#define FU_LANE_INFO_SIZE 10
///BigFusion->FusionMap->FusionMapJunction: junctions
#define FU_JUNCTIONS_SIZE 10
///BigFusion->FusionMap->FusionMapRecomLane: recom_lanes
#define FU_RECOM_LANES_SIZE 10
///BigFusion->FusionMap->FusionMapRecomLane->RecomLane: recom_lane
#define FU_RECOM_LANE_SIZE 10
///BigFusion->FusionStopLines: stopline
#define FU_STOP_LINE_SIZE 16
///BigFusion->FusionStopLines->FusionStopLine: points
#define FU_STOP_LINE_PTS_SIZE 32
///BigFusion->FusionCrosswalks: cross_walk
#define FU_CROSS_WALK_SIZE 16
///BigFusion->FusionCrosswalks->FusionCrosswalk: points
#define FU_CROSS_WALK_PTS_SIZE 200

/// interaction
///PlanningDebug: path_point
#define INTER_PLAN_PTS_SIZE 200
///PlanningDebug: path_bound
#define INTER_PLAN_PATH_BOUND_SIZE 200
///PlanningDebug: trajectory_points
#define INTER_PLAN_TRA_PTS_SIZE 200
///PlanningDebug: decisions
#define INTER_PLAN_DECISIONS_SIZE 50
///PlanningDebug: speed_plan_points
#define INTER_PLAN_SPEED_PTS_SIZE 150
///PlanningDebug: st_graphs
#define INTER_PLAN_ST_GRAPHS_SIZE 20
///PlanningDebug->SignalLightDebug: signal
#define INTER_SIGNAL_LIGHT_DEBUG_SIZE 10
///PlanningDebug->STGraph: polygon_points
#define INTER_POLYGON_PTS_SIZE 10
///PlanningDebug->HighWaySenarioDebug: main_obs_id
#define INTER_MAIN_OBS_ID_SIZE 10
///PlanningDebug->HighWaySenarioDebug: main_obs_id
#define INTER_MAIN_OBS_ID_INFO_SIZE 128
///PlanningDebug->PlanningDecision: id
#define INTER_PD_ID_SIZE 64
///PlanningDebug->SignalLightDebug->SignalDebug: light_id
#define INTER_LIGHT_ID 64
///PlanningDebug->STGraph: graph_id
#define INTER_GRAPH_ID 64
//PlanningDebug->HighWaySenarioDebug: follow_obs_id
#define INTER_FOLLOW_ID_SIZE 64
///LocalizationState: mapping_path
#define INTER_MAPING_PATH 128
///PlanningState->TakeOverReminder: reason
#define INTER_REASON_SIZE 512
///PlanningState->DecisionState->ObstacleState: main_obstacles
#define INTER_MAIN_OBS_SIZE 5
///PlanningState->DecisionState->ObstacleState: big_vehicles
#define INTER_BIG_VEHICLES_SIZE 5
///PlanningState->DecisionState->ObstacleState: cut_in_obstacles
#define INTER_BUT_OBS_SIZE 5
///PlanningState->DecisionState->ObstacleState: nudge_obstacles
#define INTER_NUUDGE_OBS_SIZE 5
///PlanningState->DecisionState->ObstacleState: lane_change_obstacles
#define INTER_LANE_CHANGE_SIZE 5
///PlanningState->FunctionState->MpaState: mr_name
#define INTER_MR_NAME_SIZE 300

/// localization
/// none

/// Perception
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

///planning
///PlanningResult: trajectory_points
#define PLANNING_TRA_PTS_SIZE 200
///PlanningResult: warnings
#define PLANNING_WARNING_SIZE 5
///PlanningResult->Warning: content
#define PLANNING_CONTENT_SIZE 256
///PlanningResult->Warning: reserved_info
#define PLANNING_INFO_SIZE 256
///PlanningResult: replan_reason
#define PLANNING_REPLAN_REASON_SIZE 512
///PlanningResult->Emergency: reason
#define PLANNING_EG_REASON_SIZE 512

///prediction
///PredictionObjects: prediction_objects
#define PRED_OBJS_SIZE 50
///PredictionObjects->PredictionObject: trajectories_in_world
#define PRED_TRA_W_SIZE 5
///PredictionObjects->PredictionObject: trajectories_in_vcs
#define PRED_TRA_V_SIZE 5
///PredictionObjects->PredictionObject->Trajectory: trajectory_points
#define PRED_TRA_PTS_SIZE 80

///routing
#define ROUT_ID_SIZE 64
///RoutingResponse->RoutingRequest: waypoints
#define ROUT_WAY_PTS_SIZE 50
///RoutingResponse->RoutingRequest: blacklisted_lanes
#define ROUT_BL_LANES_SIZE 32
///RoutingResponse->RoutingRequest: blacklisted_roads
#define ROUT_BL_ROADS_SIZE 32
///RoutingResponse->RoutingRequest: blacklisted_roads
#define ROUT_BL_ROADS_IFNO_SIZE 256
///RoutingResponse->RoutingRequest->ParkingSpace: parking_space
#define ROUT_PSPACE_PTS_SIZE 4
///RoutingResponse->RoutingRequest->LaneWaypoint: name
#define ROUT_LW_PTS_NAME_SIZE 64
///RoutingResponse: road
#define ROUT_ROAD_SIZE 50
///RoutingResponse->RoadSegment: passage
#define ROUT_PASSAGE_SIZE 5
///RoutingResponse->RoadSegment->Passage: segment
#define ROUT_SEGMENT_SIZE 20

///sensor
#define SENSOR_ID_SIZE 64
///CorrImu->Oem7Header: message_name
#define SENSOR_MESSAGE_NAME_SIZE 64
///RadarObjectArray: radar_objects
#define SENSOR_RADAR_OBJS_SIZE 100
///UssData: long_uss_data
#define SENSOR_USSDATA_LONG_SIZE 10
///UssData: short_uss_data
#define SENSOR_USSDATA_SHORT_SIZE 10

#endif
