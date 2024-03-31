#ifndef ad_INTERFACE_ad_INTERACTION_INTERFACE_H
#define ad_INTERFACE_ad_INTERACTION_INTERFACE_H

#include <stdint.h>
#include "ad_interface/ad_routing.h"
#include "ad_interface/ad_sensor.h"
#include "ad_interface/ad_world_module.h"
#include "ad_interface/ad_std.h"

namespace ad_interaction {

enum struct ParkingLotType : uint8_t {
  PRIVATE = 0,    /**< parking plot that ofen visit */
  PUBLIC,         /**< public parking plot */
};

enum struct ParkingSpaceType : uint8_t {
  UNKNOWN = 0, 
  ANGLE,            /**< Angle parking */
  PERPENDICULAR,    /**< Perpendicular parking */
  PARALLEL,         /**< Parallel parking */
  USERDEFINED,      /**< User defined unnormalized parking space (like space between cars) */
};

enum struct FollowDistanceLevel : uint8_t {
  LOW = 0,      /**< nearest follow distance*/
  MIDDLE,       /**< middle follow distance*/
  HIGH,         /**< high follow distance*/
};

struct AccSet {
  bool auto_speed_on = true;         /**< 自动设置巡航车速打开*/
  bool bend_assist_on = true;        /**< 弯道辅助打开*/
  bool take_over_assist_on = true;   /**< 超车辅助打开*/
  FollowDistanceLevel level;  /**< 跟车时距等级*/
};

struct NoaSet {
  bool auto_lane_change_on = true;     /**< 自主变道打开*/
  bool auto_nudge_on = true;           /**< 主动避让打开*/
  bool auto_ramp = true;                /**< 上下匝道打开*/
  bool close_cut_in_on = true;         /**< 防近距离加塞打开*/
  FollowDistanceLevel level;    /**< 跟车时距等级*/
};

struct Avp {
  uint64_t sequence;
  bool is_active = false;
  bool park_out = false;        /**< park in or park out */
  bool in_inwards = false;      /**< park in direction */
  bool out_left = false;        /**< park out direction */
  ParkingLotType lot_type;
  int64_t parking_lot_id; 
  ParkingSpaceType space_type;  
  int64_t parking_space_id;
};

struct Apa {
  bool is_active = false;
  bool park_out = false;        /**< park in or park out */
  bool in_inwards = false;      /**< park in direction */
  bool out_left = false;        /**< park out direction */
  ParkingSpaceType space_type;  
  int64_t parking_space_id;
};

struct Mpa {
  bool is_active = false;    /**< active signal*/
  float user_speed;          /**< 用户设置速度*/
  bool is_mapping = false;
  bool is_driving = false;
  Mpa() :
    user_speed(std::numeric_limits<float>::infinity()) {}
};

struct Noa {
  bool is_active = false;     /**< active signal*/
  NoaSet noa_set;
  float user_speed;          /**< 用户设置速度*/
  Noa() :
    user_speed(std::numeric_limits<float>::infinity()) {}
};

struct Acc {
  bool is_active = false;     /**< active signal*/
  AccSet acc_set;
  float user_speed;           /**< 用户设置速度*/
  Acc() :
    user_speed(std::numeric_limits<float>::infinity()) {}
};

struct Lcc {
  bool is_active = false;     /**< active signal*/
  AccSet acc_set;
  float user_speed;           /**< 用户设置速度*/
  Lcc() :
    user_speed(std::numeric_limits<float>::infinity()) {}
};

struct Ras {
  bool is_active = false;     /**< active signal*/
  float user_speed;           /**< 用户设置速度*/
  Ras() :
    user_speed(std::numeric_limits<float>::infinity()) {}
};

enum struct FunctionName : uint8_t {
  UNKNOWN = 0, // unknown function type
  NOA,
  ACC,
  LCC,
  AVP,
  APA,
  MPA,
  RAS,
  IDLE,
};

struct FunctionDemand {
  FunctionName name = FunctionName::NOA;
  Noa noa;      /**< 领航辅助*/
  Acc acc;      /**< 自适应巡航*/
  Lcc lcc;      /**< 车道保持*/
  Avp avp;      /**< 记忆泊车*/
  Apa apa;      /**< 泊车辅助*/
  Mpa mpa;      /**< 记忆行车*/
  Ras ras;      /**< 循迹倒车*/
};


/**
 * @brief SM/HMI interactive demand
 * 
 */

struct InteractiveDemand : public ad_std::MessageBase {
  ad_std::HeaderPOD header;                         /**< header info of this demand */
  bool demand_update = false;
  FunctionDemand function;                          /**< L2+ function demand from hmi */
};
// planning state for interactive
//***********************************************************************************

struct ErrorState {
  enum struct ErrorCode : uint8_t {
    DEPENDENCE_NOT_READY = 0,
    REFLINE_NOT_READY = 1,
  };
  bool error = false;
  ErrorCode error_code;
};

struct TurnState {
  enum struct Type : uint8_t {
    STRAIGHT = 0,       /**< 直行*/
    RIGHT = 1,          /**< 右转*/
    LEFT = 2,           /**< 左转*/
    UNTURN = 3,         /**< 调头*/
    UNKNOWN = 4,        /**< 未知*/
  };
  Type type;
  float begin_distance;
  float end_distance;
};

struct ChangeLaneState {
  enum struct Direction : uint8_t {
    RIGHT = 0,      /**< 向右变道*/
    LEFT = 1,       /**< 向左变道*/
    NONE = 2,       /**< 无变道*/
  };

  enum struct State : uint8_t {
    PREPARE = 0,      /**< 变道准备*/
    CHANGING = 1,     /**< 变道中*/
    COMPLETE = 2,     /**< 变道完成*/
    CANCELED = 3,     /**< 变道取消*/
    NONE = 4,         /**< 无变道*/
  };

  enum struct ChangeClass : uint8_t {
    DRIVER = 0,           /**< 拨杆变道*/
    NAVIGATION = 1,       /**< 导航变道*/
    EFFICIENCY = 2,       /**< 效率变道*/
    PREFERENCE = 3,       /**< 优选变道*/
    UNKNOWN = 4,
  };

  ChangeClass change_class;
  Direction type;
  State state;
};

struct NudgeState {
  enum struct Direction : uint8_t {
    RIGHT = 0,    /**< 向右绕行*/
    LEFT = 1,     /**< 向左绕行*/
    NONE = 2,     /**< 无绕行*/
  };

  enum struct State : uint8_t {
    NUDGING = 0,    /**< 绕行中*/
    NONE = 1,       /**< 无绕行*/
  };

  Direction type;
  State state;
  NudgeState()
  : type(Direction::NONE),
    state(State::NONE) {}

  void Clear() {
    type = Direction::NONE;
    state = State::NONE;
  }
};

/**
 * @brief mpa mr match state
 *
 */
struct MPAMRMatch {
  char mr_name[INTER_MR_NAME_SIZE]; /**< "路线名称" */
  bool match_status;                /**< "路线是否匹配" */
  float distance;                   /**< "车与路线起点距离" */
};

enum class Status : uint8_t {
  PASSIVE = 0,    /**< 功能抑制*/
  STANDBY,        /**< 功能就绪*/
  ACTIVE,         /**< 功能激活*/
  COMPLETE,       /**< 功能完成*/
  FAILURE,        /**< 功能失败*/
  UNKNOWN,        /**< 未知*/
};

struct NoaState {
  Status status;
};

struct AccState {
  Status status;
};

struct LccState {
  Status status;
};

struct AvpState {
  Status status;
  uint64_t sequence;
  ad_routing::RoutingResponse rout_result;
};

struct ApaState {
  Status status;
};

struct MpaState {
  Status status;
  MPAMRMatch mpa_mr_match;
};

struct RasState {
  Status status;
};

struct FunctionState {
  NoaState noa_state;   /**< 领航辅助当前状态*/
  AccState acc_state;   /**< 自适应巡航当前状态*/
  LccState lcc_state;   /**< 车道保持当前状态*/
  AvpState avp_state;   /**< 记忆泊车当前状态*/
  ApaState apa_state;   /**< 泊车辅助当前状态*/
  MpaState mpa_state;   /**< 记忆行车当前状态*/
  RasState ras_state;   /**< 循迹倒车当前状态*/
};
struct Obstacle {
  uint16_t id;                        /**< 目标唯一标识id, 范围0~2000 */
  float heading;
  ad_std::Point2f position;         /**< FLU 位置, 单位: m */
  ad_std::Point2f velocity;         /**< FLU 速度, 单位: m/s */
  ad_std::Point2f acceleration;     /** FLU 加速度, 单位: m/s2 */
  ad_world::Dimension dimension;    /**< 单位: m */
  ad_world::ObjectClass obj_class;  /**< obstacle type */
};

struct ObstacleState {
  Obstacle main_obstacles[INTER_MAIN_OBS_SIZE];           /**< main obstacles selected */
  int main_obstacles_size;
  Obstacle big_vehicles[INTER_BIG_VEHICLES_SIZE];             /**< big vehicles selected */
  int big_vehicles_size;
  Obstacle cut_in_obstacles[INTER_BUT_OBS_SIZE];         /**< cut in vehicles selected */
  int cut_in_obstacles_size;
  Obstacle nudge_obstacles[INTER_NUUDGE_OBS_SIZE];          /**< obstacles need to nudge */
  int nudge_obstacles_size;
  Obstacle lane_change_obstacles[INTER_LANE_CHANGE_SIZE];    /**< obstacles need to lane change */
  int lane_change_obstacles_size;
};

struct SpeedState {
  float current_speed;    /**< current speed of ego */
  float target_speed;     /**< target speed of ego */
  float limit_speed;      /**< road speed limit from map*/
};

struct RampJunctionState {
  enum struct RampJunctionType : uint8_t {
    DRIVING_IN = 0,
    DRIVING_OUT = 1,
    NONE = 2,
  };
  
  RampJunctionType type;  /**< 连接匝道类型 */
  float start_distance;   /**< 匝道起点离自车距离, m */
  float end_distance;     /**< 匝道终点离自车距离, m */
  RampJunctionState()
  : type(RampJunctionType::NONE),
    start_distance(std::numeric_limits<float>::infinity()),
    end_distance(std::numeric_limits<float>::infinity()) {}
};

struct DecisionState {
  ObstacleState obstacle_state;           /**< 障碍物状态信息 */
  TurnState turn_state;                   /**< 转弯状态信息 */
  ChangeLaneState change_lane_state;      /**< 变道状态信息 */
  SpeedState speed_state;                 /**< 速度状态信息 */
  NudgeState nudge_state;                 /**< 绕行状态信息 */
  RampJunctionState ramp_junction_state;  /**< 匝道状态信息 */
};

struct TakeOverReminder {

  enum struct Reason : uint8_t {
    CHANGE_LANE_FAIL = 0,     /**< 变道失败提示接管 */
    RAMP_JUNCTION_FAIL = 1,   /**< 匝道通行失败提示接管 */
    NUDGE_FAIL = 2,           /**< 绕行失败提示接管 */
    TRAFFIC_LIGHT_FAIL = 3,   /**< 信号灯通行失败提示接管 */
    REFLINE_FAIL = 4,         /**< 参考线生成失败提示接管 */
    COLLISION_RISK = 5,       /**< 碰撞风险提示接管 */
    SYSTEM_FAIL = 6,          /**< 系统问题提示接管 */
    ACC_FAIL = 7,             /**< 自适应巡航失败*/
    LCC_FAIL = 8,             /**< 车道保持失败*/
    NOA_FAIL = 9,             /**< 领航辅助失败*/
    AVP_FAIL = 10,            /**< 记忆泊车失败*/
    APA_FAIL = 11,            /**< 泊车辅助失败*/
    MPA_FAIL = 12,            /**< 记忆行车失败*/
    RAS_FAIL = 13,            /**< 循迹倒车失败*/
    UNKNOWN = 14,              /**< 未知问题 */
  };

  bool need_take_over = false;
  Reason reason;
};

/**
 * @brief voice reminder for hmi
 *
 */
struct VoiceReminder {

  enum struct Reminder : uint8_t {
    NONE = 0,                 /**< 无提示音 */
    ACC_STANDBY,              /**< “智能调速可用，请向下重拨怀挡一次激活，并松开加速踏板” */
    LCC_STANDBY,              /**< “车道居中保持可用，请向下轻拨怀挡一次激活” */
    NOA_STANDBY,              /**< “领航辅助驾驶可用，请连续向下拨怀挡两次激活” */
    ACC_ACTIVE,               /**< “已激活智能调速” */
    LCC_ACTIVE,               /**< “已激活车道居中保持” */
    NOA_ACTIVE,               /**< “已激活领航辅助驾驶” */
    ACC_FAIL,                 /**< “即将退出智能调速，请立即接管” */
    LCC_FAIL,                 /**< “即将退出车道居中保持，请立即接管” */
    NOA_FAIL,                 /**< “即将退出领航辅助驾驶，请立即接管” */
    ACC_QUIT,                 /**< “已退出智能调速” */
    LCC_QUIT,                 /**< “已退出车道居中保持” */
    NOA_QUIT,                 /**< “已退出领航辅助驾驶” */
    ACC_READY,                /**< “可松开油门，向下重拨怀挡一次，开启智能调速” */
    CHANGE_LANE_CANCELED,     /**< “变道已取消” */
    TAKE_OVER,                /**< “请立即接管” */
    CHANGE_LANE_PREPARING,    /**< "前方准备变道" */
    RAMP_JUNCTION_OUT,        /**< “即将进入匝道” */
    RAMP_JUNCTION_OUT_FAIL,   /**< “无法完成驶入匝道任务，请立即接管” */
    RAMP_JUNCTION_IN,         /**< “即将汇入主路” */
    RAMP_JUNCTION_IN_FAIL,    /**< “无法完成汇入任务，请立即接管” */
    NUDGE_FAIL,               /**< “无法完成绕行任务，请立即接管” */
    COLLISION_RISK,           /**< “车辆存在碰撞风险，请立即接管” */
    TRAFFIC_LIGHT_FAIL,       /**< “无法感知路口信号灯，请立即接管 */
    MPA_STANDYBY,             /**< "记忆行车已就绪，请选择是否使用" */
    MPA_FAIL,                 /**< "即将退出记忆行车，请立即接管" */
    MPA_QUIT,                 /**< "已退出记忆行车" */
  };

  Reminder reminder;
};

/**
 * @brief the apa move distance
 *
 */
struct ApaMoveDistance {
  bool direction;      /**< 停车0为前进，1为后退状态 */
  float distance;      /**< 前进或后退对应的距离 */
};

/**
 * @brief parking state
 * 
 */
struct ParkingState {
  enum struct ParkingStatus : uint8_t {
    PARKING = 0,
    SUCCESS = 1,
    FAIL = 2,
    UNKNOWN = 3,
  };

  ParkingStatus parking_status;        /**< 停车状态 */
  ApaMoveDistance apa_move_distance;   /**< APA前进后退距离 */
  float apa_percent;                   /**< APA进度，数值范围为0-1 */
};

/**
 * @brief the drive mode
 *
 */
enum struct DriveMode : uint8_t {
  PILOT = 0,
  CRUISE_IN = 1,
  CRUISE_OUT = 2,
  PARK_IN = 3,
  PARK_OUT = 4,
  START_RAS = 5,
  INIT = 6,
};

/**
 * @brief the signal for mode switch
 *
 */
struct SwitchMode {
  ad_std::HeaderPOD header;
  DriveMode mode;
  uint16_t parking_space_id; /**< id of parking space */
  ad_std::Point2f parking_space_left_top; /**< left-top point of parking space in vehicle coordinate system(RFU) */
  ad_std::Point2f parking_space_right_top; /**< right-top point of parking space in vehicle coordinate system(RFU) */
};

/**
 * @brief planning state
 * 
 */
struct PlanningState : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  SwitchMode switch_mode;
  ParkingState parking_state;
  ErrorState error_state;
  ad_sensor::DrivingMode driving_mode;
  FunctionState function_state;
  DecisionState decision_state;
  TakeOverReminder take_over_reminder;
  VoiceReminder voice_reminder;
};

/**
 * @brief the localization state
 *
 */
struct LocalizationState : public ad_std::MessageBase {
  enum struct State : uint8_t {
    CRUISE_SUCCESS = 0,
    CRUISE_FAIL = 1,
    PARK_SUCCESS = 2,
    PARK_FAIL = 3,
    RAS_READY = 4,
    RAS_NO_OPTIMIZATION = 5,
    UNKNOWN = 6,
  };

  ad_std::HeaderPOD header;
  SwitchMode switch_mode;
  ad_std::PoseWithCovariance pose; /**< vehicle pose in global coordinate system */
  ad_std::Point2f parking_space_left_top; /**< left-top point of parking space in global coordinate system */
  ad_std::Point2f parking_space_right_top;  /**< right-top point of parking space in global coordinate system */
  State state; /**< status of localization module */
  char mapping_path[INTER_MAPING_PATH]; /**< path of mapping results */
};
// planning debug for smartview
//******************************************************************************
struct PathPoint {
  float x;
  float y;
};

struct ADCTrajectoryPoint {
  PathPoint path_point;
  float relative_time;
  float v;
  float a;
};

struct STGraphPoint {
  float s = 1;
  float t = 2;
};

struct PathBoundPair {
  ad_std::Point2f left;
  ad_std::Point2f right;
};

enum struct PlanningDecisionType : uint8_t {
  STOP = 0,
  YIELD,
  FOLLOW,
  OVERTAKE,
  UNKNOWN_DECISION,
};

enum struct PlanningSignalColor  : uint8_t {
  RED = 0,
  GREEN,
  YELLOW,
  UNKNOWN_COLOR,
  BLACK,
};

struct PlanningDecision {
  PlanningDecisionType type;
  char id[INTER_PD_ID_SIZE];
  PathPoint pose;
  float fence_heading;
};

struct SignalDebug {
    char light_id[INTER_LIGHT_ID];
    PlanningSignalColor color;
    float remaining_time;
};

struct SignalLightDebug {
  SignalDebug signal[INTER_SIGNAL_LIGHT_DEBUG_SIZE];
  int signal_size;
};

struct STGraph {
  char graph_id[INTER_GRAPH_ID];
  STGraphPoint polygon_points[INTER_POLYGON_PTS_SIZE];
  int polygon_points_size;
};

struct PlanningLine {
  uint32_t line_id;
  float c0;
  float c1;
  float c2;
  float c3;
  float start_x;
  float end_x;
  bool is_virtual = false;
  bool is_valid = false;
};

struct HighWaySenarioDebug{
  uint32_t replan_reason ;
  uint32_t lane_change_state ;
  uint32_t speed_failure_reason;
  float lateral_error ;

  char main_obs_id[INTER_MAIN_OBS_ID_SIZE][INTER_MAIN_OBS_ID_INFO_SIZE];
  int main_obs_id_size; //第一层for循环的实际长度

  bool is_follow_obj_b = false ;
  char follow_obs_id[INTER_FOLLOW_ID_SIZE] ;
  float follow_obs_v ;
  float follow_obs_dis ;
  float follow_time_gap ;
  bool follow_cruise_b = false ;

  PlanningLine planning_left_line ;
  PlanningLine planning_right_line ;
  PlanningLine planning_left_left_line ;
  PlanningLine planning_right_right_line ;

  PlanningLine planning_left_road_edge ;
  PlanningLine planning_right_road_edge ;

  bool ego_lane_valid = false;
  bool left_lane_valid = false;
  bool right_lane_valid = false;

};

struct PlanningDebug : public ad_std::MessageBase{
  ad_std::HeaderPOD header;                 /**< header info of this demand */
  
  SignalLightDebug signal_debug;
  int signal_debug_size;
  PathPoint path_point[INTER_PLAN_PTS_SIZE];
  int path_point_size;
  PathBoundPair path_bound[INTER_PLAN_PATH_BOUND_SIZE];
  int path_bound_size;
  ADCTrajectoryPoint trajectory_points[INTER_PLAN_TRA_PTS_SIZE];
  int trajectory_points_size;
  PlanningDecision decisions[INTER_PLAN_DECISIONS_SIZE];
  int decisions_size;
  STGraphPoint speed_plan_points[INTER_PLAN_SPEED_PTS_SIZE];
  int speed_plan_points_size;
  STGraph st_graphs[INTER_PLAN_ST_GRAPHS_SIZE];
  int st_graphs_size;
  HighWaySenarioDebug  highway_scenario_debug;
};

} // namespace ad_interaction

#endif
