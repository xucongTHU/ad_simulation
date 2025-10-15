#ifndef ad_INTERFACE_ad_FUSION_INTERFACE_H
#define ad_INTERFACE_ad_FUSION_INTERFACE_H

#include <stdint.h>

#include "ad_interface/ad_std.h"

namespace ad_world {

struct FusionMeta {
  int64_t sensor_timestamp_us;
  int64_t start_timestamp_us;
  int64_t finish_timestamp_us;
};

/**
 * @brief dimension
 * 长宽高
 */
struct Dimension {
  float length;
  float width;
  float height;
};

/**
 * @brief the object class
 * 目标类型
 */
enum class ObjectClass : uint8_t {
  UNKNOWN = 0,  /**< 未知 */
  BIG_CAR,     /**< 大型车辆，包含卡车、大巴以及大尺寸特种车辆 */
  CAR,         /**< 汽车*/
  MOTOR_BIKE,      /**< 摩托车 */
  BICYCLE,         /**< 自行车 */
  PEDESTRIAN,   /**< 行人 */
  CONE,         /**锥桶 */
  WATER_SAFETY_BARRIERS, /**水马 */
};

/**
 * @brief the object in lane
 * 融合目标所在的车道
 */
enum class ObjectInLane : uint8_t {
  NOT_ASSIGNED = 0,  /**< 未指定的 */
  EGO_LANE,     /**< 本车所在车道 */
  ADJACENT_LEFT_LANE_L1,         /**< 左侧车道L1 */
  ADJACENT_RIGHT_LANE_R1,      /**< 右侧车道R1 */
  NEXT_ADJACENT_LEFT_LANE_L2,         /**< 左左侧车道L2 */
  NEXT_ADJACENT_RIGHT_LANE_R2,   /**< 右右侧车道R2 */
};

/**
 * @brief the object motion status
 * 目标的运动状态属性
 */
enum class MotionStatus : uint8_t {
  INVALID = 0,  /**< 不可用 */
  UNKNOWN,     /**< 未知 */
  MOVING,         /**< 运动的障碍物(V ≥ 0.5 m/s)*/
  STOPPED,      /**< 运动停止的障碍物 */
  MOVING_SLOWLY,         /**< 低速运动的障碍物(V ＜ 0.5 m/s) */
  STATIONARY,   /**< 长期静止的障碍物(V < 0.2m/s) */
};

/**
 * @brief the turn light
 * 目标转向灯的状态
 */
enum class TurnLight : uint8_t {
  NO_DIRECTION_INDICATOR = 0,  /**<  no direction indicator 无转向灯*/
  LEFT_INDICATOR,     /**< 左转灯 */
  RIGHT_INDICATOR,         /**< 右转灯*/
  WARMING_INDICATOR,      /**< 双闪 */
  UNKNOWN,         /**< 未知 */
};

/**
 * @brief the brake light
 * 目标的刹车灯状态
 */
enum class BrakeLight : uint8_t {
  OFF = 0,  /**<  目标刹车灯关闭 */
  ON,     /**< 目标刹车灯打开 */
  UNKNOWN,         /**< 未知 */
};

/**
 * @brief the head orientation
 * 目标最小外包框的水平朝向的状态
 */
enum class HeadOrientation : uint8_t {
  INVALID = 0,  /**<  invalid 不可信 */
  DRIFTING_RIGHT,     /**< Drifting Right 向右转*/
  CROSSING_RIGHT,         /**< Crossing Right 向右横穿 */
  ONCOMING_DRIFTING_RIGHT,         /**< oncoming Drifting Right 相向而行的右转 */
  ONCOMING,         /**< oncoming 相向而行 */
  ONCOMING_DRIFTING_LEFT,         /**< oncoming Drifting Left 相向而行的左转 */
  CROSSING_LEFT,         /**<  Crossing Left 向左横穿 */
  DRIFTING_LEFT,         /**< Drifting Left 向左转 */
  PRECEEDING /**< precedding  前进的*/
};

/**
 * @brief the track type
 * 目标融合数据来源的传感器类型
 */
enum class TrackType : uint8_t {
  CAMERA = 0,  /**<  仅相机参与(正常状态) */
  RADAR,            /**< 仅毫米波雷达参与 */
  CAMERA_RADAR,         /**< 相机与毫米波雷达融合 */
  USS,         /**< 仅超声波参与 */
  CAMERA_USS,         /**< 相机与超声波雷达参与 */
  RADAR_USS,            /**< 超声波与毫米波雷达参与 */
  CAMERA_RADAR_USS,         /**<  相机与超声波雷达与毫米波雷达共同参与 */
};

/**
 * @brief Fusion Objects Tab
 *
 */
struct FusionObjectTab {
  uint16_t id; /**< 目标唯一标识id, 范围0~2000 */
  ad_std::Point2f position; /**< 目标最小外包框几何中心在VCS坐标系下的x轴与y轴方向的位置, 单位: m  */
  ad_std::Point2f velocity; /**< 目标最小外包框几何中心在VCS坐标系下的x轴与y轴方向的绝对速度, 单位: m/s */
  ad_std::Point2f acceleration; /** 目标最小外包框几何中心在VCS坐标系下的x轴与y轴方向的绝对加速度, 单位: m/s2 */
  ad_std::Point2f position_variance_xy; /**< 目标最小外包框几何中心在VCS坐标系下的x坐标与y坐标的方差 */
  ad_std::Point2f position_covariance_xy; /**< 目标最小外包框几何中心在VCS坐标系下的x坐标和y坐标的协方差 */
  Dimension dimension; /**< 目标最小外包框的长度、宽度、高度，分别为(length, width, height), 单位: m */
  ObjectClass obj_class; /**< 目标类型 */
  ObjectInLane in_lane; /**< 融合目标所在的车道 */

  /** 融合目标状态 */
  // bit0:ACC候选+选中目标
  // bit1:AEB候选目标
  // bit2:FCW_VD_Alert
  // bit3:FCW_VRU_Alert
  // bit4:AEBPrefill_VRU_Alert
  // bit5:AEB_VD_Alert
  // bit6:AEB_VRU_Alert
  // bit7:ACC选中目标
  // bit8:JAPrefill_VD_Alert
  // bit9:JAPrefill_VRU_Alert
  // bit10:AEBJA_VD_Alert
  // bit11:AEBJA_VRU_Alert
  // bit12:AEB_ID(包含FCW,AEBprefill,AEB)
  // bit13:JA_VRU_ID
  // bit14:JA_VD_ID
  // bit15:JA_prefill or JA (prefill:0, JA:1)
  uint16_t status;

  uint64_t age; /**< 目标的跟踪时间, us*/
  MotionStatus motion_status; /**< 目标的运动状态属性 */
  TurnLight turn_light; /**< 目标转向灯的状态  */
  BrakeLight brake_light; /**< 目标的刹车灯状态 */
  float heading_angle; /**< 目标最小外包框在VCS坐标系下的水平朝向角，单位：弧度 */
  HeadOrientation head_orientation; /**< 目标最小外包框的水平朝向的状态 */
  TrackType track_type; /**< 目标融合数据来源的传感器类型 */
};

/**
 * @brief Fusion Objects Tab
 *
 */
struct FusionObjectsTab {
  ad_std::HeaderPOD header; /**< header */
  FusionObjectTab fusion_object_tab[FU_OBJ_TAB_SIZE]; /**< 融合目标障碍物 */
  int fusion_object_tab_size;
  uint16_t num_of_obj; /**< 融合总输出结果中的感知目标个数, 单位:个, 数量范围0~999 */
};

/**
 * @brief the lane line class
 * 车道线类别
 */
enum class LaneLineClass : uint8_t {
  FRONT = 0,          /**< front 前 */
  BACK,                    /**< back 后 */
  ROAD_EDGE,      /**< road edge 道路边界 */
};

/**
 * @brief fusion lane
 *
 */
struct FusionLaneCubicPolynomial {
  float start_x; /**< 车道线起始x */
  float end_x;  /**< 车道线终止x */
  float c0;          /**< 车道线系数c0 */
  float c1;          /**< 车道线系数c1 */
  float c2;          /**< 车道线系数c2 */
  float c3;          /**< 车道线系数c3 */
};

/**
 * @brief the lane line type
 * 车道线类型
 */
enum class LaneLineType : uint8_t {
  UNKNOWN = 0,  /**< unknown 未知 */
  SOLOD,            /**< solid 实线 */
  DASHED,         /**< dashed 虚线 */
  LEFT_DASHED_RIGHT_SOLID,         /**< left dashed right solid 左虚右实线 */
  LEFT_SOLID_RIGHT_DASHED,         /**< left solid right dashed  左实右虚线 */
  DOUBLE_DASHED,         /**< double dashed  双虚线 */
  DOUBLE_SOLID,         /**< double solid  双实线 */
  BARRIER,         /**< barrier */
  BOTTS,        /**< botts圆点凸点车道 */
  CURB,         /**< Curb路缘(小于25cm，大于等于10cm)*/
  CONES_POLES,         /**< Cones_poles 锥体或杆线 */
  INVALID,         /**< invalid 不可信 */
};

/**
 * @brief the lane index
 *
 */
enum class LaneIndex : uint8_t {
  LANE_NOT_ASSIGNED = 0, /**<lane not assigned */
  LANE_LKA_LEFT,         /**<lane lka left */
  LANE_LKA_RIGHT,        /**<lane lka right */
  LANE_NEXT_LEFT,        /**<lane next left */
  LANE_NEXT_RIGHT,       /**<lane next right */
  LANE_NEXT_NEXT_LEFT,   /**<lane next next left */
  LANE_NEXT_NEXT_RIGHT,   /**<lane next next right */
  VIRTUAL_LANE_LKA_LEFT,
  VIRTUAL_LANE_LKA_RIGHT,
};

/**
 * @brief fusion lane
 *
 */
struct FusionLane {
  /** 车道线的id标识 
   * FusionFrontDesireLane_L0，ID可以为5，图中由红色标出。
   * 为融合模块结合道路实际情况与车辆routing拟合出车辆的期望的车道线，左侧车道线，采用多项式系数表达；
   * FusionFrontDesireLane_L1，ID可以为6，图中由红色标出。
   * 融合模块结合道路实际情况与车辆routing拟合出车辆的期望的车道线，左侧车道线，采用多项式系数表达；
   * 自车当前车道的左侧第1条车道线id号为2, FusionFrontLaneLine front_lane_L0；
   * 自车当前车道的左侧第2条车道线id号为1, FusionFrontLaneLine front_lane_L1；
   * 自车当前车道的右侧第1条车道线id号为3, FusionFrontLaneLine front_lane_R0；
   * 自车当前车道的右侧第2条车道线id号为4；以此类推。
  */
  uint8_t id;
  LaneLineClass line_class; /**< 车道线类型 */
  FusionLaneCubicPolynomial poly_array[FU_LANE_CUBICPOLY_SIZE]; /**< VCS坐标系下，车道线曲线公式f(x)=C0+C1*x+C2*x^2+C3*x^3中的系数C0 */
  uint8_t poly_num;
  float line_range; /**< 车道线的长度, 单位: m */
  float line_detect_quality; /**< 车道线的质量 [high score>0.9, medium 0.7-0.9, low2 0.4-0.7, low1 0.0-0.4]*/

  /** 道路的类型 */
  // bit0:LAP_Is_Construction_Area   1 "TRUE" 0 "FALSE"
  // bit1:LAP_Is_Highway_Exit_Left   1 "TRUE" 0 "FALSE"
  // bit2:LAP_Is_Highway_Exit_Right  1 "TRUE" 0 "FALSE"
  // bit3:LAP_Is_Highway_Entrance_Left   1 "TRUE" 0 "FALSE"
  // bit4:LAP_Is_Highway_Entrance_Right   1 "TRUE" 0 "FALSE"
  // bit5:LAP_Is_Highway_Merge_Left  1 "TRUE" 0 "FALSE"
  // bit6:LAP_Is_Highway_Merge_Right 1 "TRUE" 0 "FALSE"
  // bit7:COM_Is_HighSpeed_Road      1 "TRUE" 0 "FALSE"
  // bit8: Urban_TurnLeft_Road     1 "TRUE" 0 "FALSE"
  // bit9: Urban_TurnRight_Road   1 "TRUE" 0 "FALSE"
  // bit10: Urban_Straight_Road   1 "TRUE" 0 "FALSE"
  // bit11:Reserved
  uint8_t road_type;
  LaneLineType lane_line_type; /** <车道线的类型 */
  float line_mark_width; /**< 线的宽度 */
  LaneIndex index;
};

/**
 * @brief fusion lanes
 *
 */
struct FusionLanes {
  ad_std::HeaderPOD header; /**< Header信息 */
  FusionMeta meta; /**< FusionMeta信息 */
  FusionLane lanes[FU_LANES_SIZE]; /**< 车道线集合容器 */
  int lanes_size;
};

/**
 * @brief the traffic light color
 * 交通灯颜色
 */
enum class TrafficLightColor : uint8_t {
  UNKNOWN = 0,  /**< unknown未知 */
  GREEN,            /**< green */
  YELLOW,         /**< yellow */
  RED,                 /**< red */
  black,               /**< black */
};

/**
 * @brief the traffic light type
 * 交通灯类型
 */
enum class TrafficLightType : uint8_t {
  UNKNOWN = 0,  /**< unknown未知 */
  LEFT_ARROW,            /**< left arrow 左箭头 */
  RIGHT_ARROW,         /**< right arrow 右箭头 */
  FORWARD_ARROW,                 /**< forward arrow 前进箭头*/
  PED,               /**< ped 行人 */
  TURN_ARROUND, /**< turn arround 掉头 */
};

/**
 * @brief fusion traffic light
 *
 */
struct FusionTrafficLight {
  uint32_t id; /**< id */
  TrafficLightColor color;  /**< 红绿灯颜色 */
  TrafficLightType type; /**< 交通灯类型 */
  float confidence; /**< 范围[0, 1] 1代表概率最高 */
  bool blink; /**< traffic light blikn 0：true, 1：false */
  float remaining_time; /**< traffic light remaining time. s 剩余时间*/
};

struct FusionTrafficLights {
  ad_std::HeaderPOD header; /**< header */
  FusionTrafficLight trafficlights[FU_TRAFFICLIGHTS_SIZE];
  int trafficlights_size;
};
/**
 * @brief traffic sign
 *
 */
struct TrafficSign {
  /** 
   * 0x1:HIGH_SPEED_LIMIT_5
   * 0x2:HIGH_SPEED_LIMIT_10
   * 0x3:HIGH_SPEED_LIMIT_15
   * 0x4:HIGH_SPEED_LIMIT_20
   * 0x5:HIGH_SPEED_LIMIT_25
   * 0x6:HIGH_SPEED_LIMIT_30
   * 0x7:HIGH_SPEED_LIMIT_35
   * 0x8:HIGH_SPEED_LIMIT_40
   * 0x9:HIGH_SPEED_LIMIT_45
   * 0x10:HIGH_SPEED_LIMIT_50
   * 0x11:HIGH_SPEED_LIMIT_55
   * 0x12:HIGH_SPEED_LIMIT_60
   * 0x13:HIGH_SPEED_LIMIT_65
   * 0x14:HIGH_SPEED_LIMIT_70
   * 0x15:HIGH_SPEED_LIMIT_75
   * 0x16:HIGH_SPEED_LIMIT_80
   * 0x17:HIGH_SPEED_LIMIT_85
   * 0x18:HIGH_SPEED_LIMIT_90
   * 0x19:HIGH_SPEED_LIMIT_95
   * 0x20:HIGH_SPEED_LIMIT_100
   * 0x21:HIGH_SPEED_LIMIT_105
   * 0x22:HIGH_SPEED_LIMIT_110
   * 0x23:HIGH_SPEED_LIMIT_115
   * 0x24:HIGH_SPEED_LIMIT_120
   * 0x25:WARNING_CONSTRUCTION
   * 0x26:WARNING_ROADS
   * 0x27:WARNING_UPHILL
   * 0x28:WARNING_DOWNHILL
   * 0x29:WARNING_SLIP
   * 0x30:WARNING_STONE
   * 0x31:WARNING_LIGHT
   * 0x32:WARNING_WIND
   * 0x33:WARNING_DANGER
   * 0x34:WARNING_PERSON
   * 0x35:WARNING_CHILDREN
   * 0x36:PROHIBIT_OVERTAKE
   * 0x37:LIFTPROHIBIT_OVERTAKE
   * 0x38:PROHIBIT_TURN_ARROUND
   * 0x39:PROHIBIT_TURN_RIGHT
   * 0x40:PROHIBIT_TURN_LEFT
   * 0x41:PROHIBIT_STOP
   * 0x42:PROHIBIT_CAR_THROUGH
   * 0x43:PROHIBIT_THROUGH
   * 0x44:HIGH_SPEED_LIMIT_140
   * 0x45:HIGH_SPEED_LIMIT_141
   * 0x46:PROHIBIT_STOP
   * 0x47:PROHIBIT_CAR_THROUGH
   * 0x48:PROHIBIT_THROUGH
   * 0x49:PROHIBIT_PARKING
   * 0x50:PROHIBIT_OTHER
   * 0x51:WARNING_OTHER_CONSTRUCTION
   * 0x52:WARNING_OTHER
   * 0x53:INDICATION
   * 0x54:BLUE_WHITE
   * 0x55:WHITE_BLACK
   * 0x56:GREEN_WHITE
   * 0x57:TRAVEL
   * 0x58:RED_WHITE
   * 0x59:YELLOW_BLACK
   * 0x60:LOW_SPEED_LIMIT50
   * 0x61:LOW_SPEED_LIMIT60
   * 0x62:LOW_SPEED_LIMIT70
   * 0x63:LOW_SPEED_LIMIT80
   * 0x64:LOW_SPEED_LIMIT900x66:LOW_SPEED_LIMIT110
   * 0x65:LOW_SPEED_LIMIT100
   * 0x67:DIVERSIONS
   * 0x68:MULTI_SIGNS
   * 0x69:UNKNOWN
   * 
  */
  uint8_t type;
  ad_std::Point3f pose_world; /**< 交通标志牌在世界坐标系的(x, y, z)坐标 */
};

/**
 * @brief fusion traffic sign
 *
 */
struct FusionTrafficSign {
  ad_std::HeaderPOD header; /**< header */
  FusionMeta meta; /**< meta info */
  uint8_t size; /**< actual used size */
  TrafficSign traffic_signs[FU_TRAFFIC_SIGNS_SIZE]; /**< traffic sign array */
  int traffic_signs_size;
};

/**
 * @brief space
 *
 */
struct Space {
  ad_std::Point2f parkingspace_left_up_point; /**<left-up point of parkingslot in IPM coordinate system*/
  ad_std::Point2f parkingspace_right_up_point; /**<right-up point of parkingslot in IPM coordinate system*/
  ad_std::Point2f parkingspace_left_down_point; /**<left-down point of parkingslot in IPM coordinate system*/
  ad_std::Point2f parkingspace_right_down_point; /**<right-down point of parkingslot in IPM coordinate system*/
  ad_std::Point2f chock1_point;  /**<chock point1, point adjacent to the one side that gotten from segmentation in the direction of parkingslot entrance*/
  ad_std::Point2f chock2_point; /**<chock point2, point adjacent to the another side that gotten from segmentation in the direction of parkingslot entrance*/
  float angle1;     /**<angle about left parking-line and positive direction of x-axis*/
  float angle2;     /**<angle about right parking-line and positive direction of x-axis*/
  bool left_up_point_occ_statuscar;      /**<binary,1 for left-up point sheltered from owb car,0 for left-up point not sheltered from owb car */
  bool left_up_point_occ_by_obstacle;    /**<binary,1 for left-up point sheltered from obstacle,0 for left-up point not sheltered from obstacle */
  uint8_t  left_up_point_camera_source;  /**<which camera does the left-up point appear in ,1 for front camera,2 for back camera, 3 for left camera, 4 for right camera */
  bool right_up_point_occ_by_car;     /**<binary,1 for right-up point sheltered from owb car,0 for right-up point not sheltered from owb car */
  bool right_up_point_occ_by_obstacle;  /**<binary,1 for right-up point sheltered from obstacle,0 for right-up point not sheltered from obstacle */
  uint8_t  right_up_point_camera_source;  /**<which camera does the right-up point appear in ,1 for front camera,2 for back camera, 3 for left camera, 4 for right camera */
};

/**
 * @brief the park slot type
 * 车位类型
 */
enum class ParkSlotType : uint8_t {
  UNKNOWN = 0,  /**< unknown未知 */
  LINE_MARK_VERTICAL,            /**< line mark vertical 画线车位垂直 */
  LINE_MARK_HORIZONTAL,         /**< line mark horizontal 画线车位水平 */
  LINE_MARK_DIAGONAL,                 /**< line mark diagonal 画线车位斜向 */
  SPATIAL_VERTICAL,               /**< spatial vertical 空间车位垂直 */
  SPATIAL_HORIZONTAL,       /**< spatial horizontal 空间车位水平 */
};

/**
 * @brief park space
 *
 */
struct ParkingSpace {
  int64_t parking_camera_timestamp;
  uint32_t track_id;
  uint32_t lost_age;
  Space space_vcs;  /**< parking space center point of parkingslot in VCS coordinate system*/
  Space space_ipm; /**< parking space center point of parkingslot in IPM coordinate system*/
  float score;     /**< confidence score about the parkingspace detected results, between 0 and 1 */ 
  ParkSlotType type;   /**< parkingspace type,1 for vertical, 2 for parallel, 3 for slant */
  uint8_t occu;   /**< binary,1 for occupy,0 for not occupy */
  char psd_id[FU_PSD_ID_SIZE];
  bool narrow;    /** 1 for narrow parkspace, 0 for not */
};
/**
 * @brief fusion park space
 *
 */
struct FusionParkSpace {
  ad_std::HeaderPOD header;
  ParkingSpace parking_space[FU_PARKING_SPACE_SIZE]; /**< parkingspaces*/
  int parking_space_size;
};
/**
 * @brief the freespace point type
 * 可行驶区域点类型
 */
enum class FreespacePointType : uint8_t {
  BACKGROUND = 0,  /**< back ground */
  CURB,            /**< curb 路牙子 */
  OBSTACLES,         /**< obstacles 障碍物 */
};

/**
 * @brief free space point
 *
 */
struct FreespacePoint {
  ad_std::Point2f vcs_point; /**< 需要包含的视野,覆盖车头X正方向20m, 车身坐标系, m, 可行驶区域坐标点 **/
  FreespacePointType type; /**< 可行驶区域点的类别 */
};

/**
 * @brief fusion free space
 *
 */
struct FusionFreeSpace {
  ad_std::HeaderPOD header;
  FreespacePoint freespace_points[FU_FS_POINTS_SIZE]; /**< 多个区域点组成 */
  int freespace_points_size;
};

struct StopLineCubicPolynomial {
  float start_x; /**< 车道线起始x */
  float end_x;  /**< 车道线终止x */
  float c0;          /**< 车道线系数c0 */
  float c1;          /**< 车道线系数c1 */
  float c2;          /**< 车道线系数c2 */
  float c3;          /**< 车道线系数c3 */
};

/**
 * @brief the stop line type
 * 停止线类型
 */
enum class StoplineType : uint8_t {
  IS_STOP_LINE = 0,                          /**< 停止线 */
  IS_PARKING_YIELD_LINE,                     /**< 停车让行线 */
  IS_DECELERATION_YIELD_LINE,                /**< 减速让行线 */
  IS_VIRTUAL_STOP_LINE,                      /**< 虚拟停止线 */
};

struct FusionStopLine {
  uint64_t avaliable;
  uint32_t id;
  double score;
  bool is_predict;
  float width;
  ad_std::Point2d left_end_point;
  ad_std::Point2d right_end_point;
  StopLineCubicPolynomial poly;
  ad_std::Point2d points[FU_STOP_LINE_PTS_SIZE];
  int points_size;
  StoplineType stop_line_type;
};

struct FusionStopLines {
  ad_std::HeaderPOD header; /**< header */
  FusionMeta meta; /**< meta info */
  uint32_t size; /**< actual used size */
  FusionStopLine stopline[FU_STOP_LINE_SIZE];
  int stopline_size;
};
enum class BEVLaneClass : uint8_t {
  UNKNOWN,
  BOUNDRAY,
  DIVIDE,
  CROSS_WALK,
  STOP_LINE,
  OTHER,
  TYPE_INVALID,
};

struct LaneCubicPolynomial {
  /**the lane start point x , the closest point is start_x, include front and
   * rear, 0.1 or -0.1 */
  float start_x;
  /**the lane end point x, the furth point is end_x, include front and rear,
   * 1000 or -1000 */
  float end_x;
  float c0;
  float c1;
  float c2;
  float c3;
};

enum class LaneColor : uint8_t {
  UNKNOWN = 0,
  WHITE,  /**<lane white */
  YELLOW, /**<lane yello */
};

enum class LaneType : uint8_t {
  UNKNOWN = 0,             /**<lane type unknown */
  SOLID,                   /**<lane solid */
  ROAD_EDGE,               /**<lane road edge */
  DASHED,                  /**<lane dashed */
  LEFT_DASHED_RIGHT_SOLID, /**<lane left dashed right solid */
  LEFT_SOLID_RIGHT_DASHED, /**<lane left solid right dashed */
  DOUBLE_DASHED,           /**<lane double dashed */
  DOUBLE_SOLID,            /**<lane double solid */
  BARRIER,                 /**<lane barrier */
  TYPE_INVALID             /**<lane type invalid */
};

struct FusionCrosswalk {
  uint64_t avaliable;       /**<binary,1 for available,0 for unavailable */
  BEVLaneClass cls;            /**<the lane class */
  double score;             /**<the lane score */
  LaneCubicPolynomial poly; /**<the lane vcs poly */
  uint8_t index;               /**<reserved*/
  LaneColor color;          /**the lane color type*/
  LaneType type;            /**<the lane type */
  ad_std::Point2d points[FU_CROSS_WALK_PTS_SIZE]; /**<the lane vcs points */
  int points_size;
};

struct FusionCrosswalks {
  ad_std::HeaderPOD header; /**< header */
  FusionMeta meta; /**< meta info */
  uint32_t size; /**< actual used size */
  FusionCrosswalk cross_walk[FU_CROSS_WALK_SIZE];
  int cross_walk_size;
};

/**
 * @brief the ramp junction type
 * 连接匝道类型
 */
enum class RampJunctionType : uint8_t {
  JUNCTION_UP_DRIVING_IN = 0,       /**< 连接上匝道(驶入) */
  JUNCTION_DOWN_DRIVING_OUT,  /**< 连接下匝道(驶出) */
  HIGH_SPEED_DRIVING_IN,                 /**< 高速间匝道(驶入) */
  HIGH_SPEED_DRIVING_OUT,             /**< 高速间匝道(驶出) */
};

/**
 * @brief Ramp Junction
 */
struct RampJunction {
  uint32_t number; /**< 匝道编号, 离自车由近及远, 从1开始顺序编号*/
  RampJunctionType type; /**< 连接匝道类型 */
  uint32_t distance; /**< 匝道起点离自车距离, cm */
  uint32_t end_distance; /**< 匝道终点离自车距离, cm */
};

/**
 * @brief fusion map ramp junction
 * 融合匝道
 */
struct FusionMapRampJunction {
  RampJunction ramp_junctions[FU_RAMP_JUNCTIONS_SIZE];
  int ramp_junctions_size;
};

/**
 * @brief stop line
 */
struct Stopline {
  uint32_t number; /**< 停止线编号, 离自车由近及远, 从1开始顺序编号*/
  ad_std::Point2d shape[FU_STOPLINE_SHAPE_PTS_SIZE]; /**< 几何形点串 */
  int shape_size;
  StoplineType type; /**< 停止线类型 */
  uint32_t co_light_id; /**< 关联的交通灯ID */
  uint32_t co_lane_id[FU_STOPLINE_COLANE_SIZE]; /**< 关联的车道ID */
  int co_lane_id_size;
  uint32_t distance; /**< 停止线离自车距离, cm */
};

/**
 * @brief fusion map stop line
 * 融合地图停止线
 */
struct FusionMapStopline {
  Stopline stop_lines[FU_STOPLINES_SIZE];
  int stop_lines_size;
};

/**
 * @brief the traffic light type
 * 信号灯类型
 */
enum class MapTrafficLightType : uint8_t {
  ROAD_INTERSECTION = 0,                              /**< 道口信号灯 */
  RAMP_JUNCTION,               /**< 匝道信号灯 */
  TOLL_STATION,  /**< 收费站信号灯 */
  PEDESTRIAN_CROSSING_PED,                 /**< 人行横道行人信号灯 */
  PEDESTRIAN_CROSSING_MOTOR,                 /**< 人行横道机动车信号灯*/
  NON_MOTORIZED_LANE,                 /**< 非机动车道信号灯*/
  TUNNEL,                 /**< 隧道信号灯*/
  BRIDGE,                 /**< 桥梁信号灯*/
  LANE_CONTROL,                 /**< 车道管制信号灯*/
  RAILWAY,                 /**< 铁路信号灯*/
  TRAM,                 /**< 有轨电车信号灯*/
  DIRECTION_INDICATION,                 /**< 方向指示信号灯*/
  FLASHING_WARNING,                 /**< 闪光警告灯*/
  TIMER,                 /**< 计时器*/
};

/**
 * @brief the lateral position
 * 方位
 */
enum class LateralPosition : uint8_t {
  UNKNOWN = 0,                              /**< unknown */
  RIGHT,               /**< right */
  LEFT,  /**< left */
  ABOVE,                 /**< above */
  SURFACE,                 /**< surface */
};

/**
 * @brief the construction type
 * 交通灯安装位置
 */
enum class ConstructionType : uint8_t {
  UNKNOWN = 0,             /**< unknown */
  HORIZONTAL,               /**< Horizontal */
  VERTICAL,                       /**< Vertical */
  DOGHOUSE,                  /**< Doghouse */
};

/**
 * @brief the turn type
 * 控制转向信息
 */
enum class TurnType : uint8_t {
  GO_STRAIGHT = 0,                                                                        /**< 直行 */
  TURN_RIGHT,                                                                                  /**< 右转 */
  GO_STRAIGHT_OR_TURN_RIGHT,                                         /**< 直行或右转 */
  TURN_LEFT,                                                                                     /**< 左转 */
  GO_STRAIGHT_OR_TURN_LEFT,                                            /**< 直行或左转 */
  TURN_LEFT_OR_TURN_RIGHT,                                              /**< 左转或右转 */
  GO_STRAIGHT_OR_TURN_LEFT_OR_TURN_RIGHT,     /**< 直行或左转或右转 */
  TURN_AROUND,                                                                            /**< 掉头 */
  GO_STRAIGHT_OR_TURN_AROUND,                                   /**< 直行或掉头 */
  TURN_LEFT_OR_TURN_AROUND,                                        /**< 左转或掉头 */
  LEFT_FORWARD,                              /**< 左前方 */
  MID,                                       /**< 中间 */
  RIGHT_FORWARD,                             /**< 右前方 */
};

/**
 * @brief traffic light
 */
struct TrafficLight {
  uint32_t number; /**< 交通灯编号, 离自车由近及远, 从1开始顺序编号*/
  ad_std::Point2d point; /**< 形点坐标 */
  MapTrafficLightType type; /**< 信号灯类型 */
  LateralPosition position; /**< 方位*/
  ConstructionType construct_type; /**< 交通灯安装位置*/
  ad_std::Point2d bounding_box[FU_TLIGHT_BBOX_PTS_SIZE]; /**< 外包围矩形形点串 */
  int bounding_box_size;
  TurnType turn_type; /**< 控制转向信息*/
  uint32_t co_lane_id[FU_TLIGHT_COLANE_SIZE]; /**< 关联的车道ID */
  int co_lane_id_size;
  uint32_t distance; /**< 交通灯离自车距离, cm */
};

/**
 * @brief fusion map traffic light
 * 融合地图交通灯
 */
struct FusionMapTrafficLight {
  TrafficLight traffic_lights[FU_TLIGHTS_SIZE];
  int traffic_lights_size;
};
/**
 * @brief the transition type
 * 过渡车道类型
 */
enum class TransitionType : uint8_t {
  OPENING = 0,                                  /**< Opening车道形成 */
  CLOSING,                                          /**< Closing车道结束 */
  MERGING,                                         /**< Merging车道合流 */
  SPLITTING,                                      /**< Splitting车道分流 */
};

/**
 * @brief transition
 */
struct Transition {
  uint32_t number; /**< 过渡区域编号, 离自车由近及远, 从1开始顺序编号*/
  uint32_t lane_number; /**< 过渡区域所在车道编号, 从右到左, 从1开始顺序编号 */
  TransitionType type; /**< 过渡车道类型 */
  uint32_t distance; /**< 过渡起点离自车距离, cm */
  uint32_t end_distance; /**< 过渡终点离自车距离, cm */
  TurnType turn_type;         /**< 车道转向类型*/
};

/**
 * @brief fusion map transition
 *  融合地图过渡区
 */
struct FusionMapTransition {
  Transition transitions[FU_TRANSITIONS_SIZE];
  int transitions_size;
};
/**
 * @brief crosswalk
 */
struct Crosswalk {
  uint32_t number; /**< 人行横道编号, 离自车由近及远, 从1开始顺序编号*/
  ad_std::Point2d shape[FU_CROSSWALK_SHAPE_SIZE]; /**< 几何形点串 */
  int shape_size;
  uint32_t distance; /**< 人行横道离自车距离, cm */
  uint32_t end_distance;
};
/**
 * @brief fusion map crosswalk
 *  融合地图人行横道
 */
struct FusionMapCrosswalk {
  Crosswalk crosswalk[FU_CROSSWALK_SIZE];
  int crosswalk_size;
};
/**
 * @brief fusion map crosswalk
 *  融合地图人行横道
 */
struct LaneWidth {
  uint32_t number;                    /**< 当前所在车道编号, 从右到左, 从1开始顺序编号 */
  uint32_t left_lane_width;    /**< 左侧车道宽度, cm */
  uint32_t cur_lane_width;    /**< 当前车道宽度, cm */
  uint32_t right_lane_width; /**< 右侧车道宽度, cm */
};

/**
 * @brief wait area
 *  待转区
 */
struct WaitArea {
  uint32_t number;                    /**< 待转区编号, 离自车由近及远, 从1开始顺序编号 */
  uint32_t lane_number;          /**< 待转区所在车道编号, 从右到左, 从1开始顺序编号 */
  uint32_t distance; /**< 待转区起点离自车距离, cm */
  uint32_t end_distance; /**< 待转区终点离自车距离, cm */
};

/**
 * @brief fusion map wait area
 *  融合地图待转区
 */
struct FusionMapWaitArea {
  WaitArea wait_area[FU_WAIT_AREA_SIZE];
  int wait_area_size;
};
/**
 * @brief lane info section
 *  路段
 */
struct LaneInfo {
  uint32_t lane_number;                       /**< 车道编号, 从右到左, 从1开始顺序编号 */
  uint32_t value_min_speed;              /**< 车道最小限速, KpH  (kilometre per hour) */
  uint32_t value_max_speed;             /**< 车道最大限速, KpH  (kilometre per hour) */
  uint32_t value_exper_speed;          /**< 车道经验速度, KpH  (kilometre per hour) */
  TurnType type;                                       /**< 车道转向类型*/
  ad_std::Point2d start_point;         /**< 车道起点坐标*/
  ad_std::Point2d end_point;          /**< 车道终点坐标*/
};

/**
 * @brief lane info section
 *  路段
 */
struct LaneInfoSection {
  uint32_t number;                                 /**< 路段(车道组)编号(离自车由近及远, 从1开始顺序编号) */
  uint32_t lane_count;                          /**< 车道数量 */
  LaneInfo lane_info[FU_LANE_INFO_SIZE];  /**< 车道信息 */
  int lane_info_size;
  uint32_t distance;                                /**< 路段(车道组)离自车距离, cm */
};

/**
 * @brief fusion map lane info section
 *  融合地图路段
 */
struct FusionMapLaneInfoSection {
  LaneInfoSection laneinfo_section[FU_LANEINFO_SECTION_SIZE];
  int laneinfo_section_size;
};
/**
 * @brief junction
 *  路段
 */
struct Junction {
  uint32_t number;                /**< 路口编号, 离自车由近及远, 从1开始顺序编号 */
  TurnType type;                     /**< 车道转向类型*/
  uint32_t distance;               /**< 路口起点离自车距离, cm*/
  uint32_t end_distance;     /**< 路口终点离自车距离, cm*/
};

/**
 * @brief fusion map junction
 *  融合地图路口
 */
struct FusionMapJunction {
  Junction junctions[FU_JUNCTIONS_SIZE];
  int junctions_size;
};

/**
 * @brief remain lane dis
 *  路段
 */
struct RemainLaneDis {
  uint32_t current_lane;                /**< 当前所在车道的编号, 从右到左, 从1开始顺序编号 */
  uint32_t remain_dis;                   /**< 车道剩余距离, cm*/
};

/**
 * @brief remain lane dis
 *  路段
 */
struct RecomLane {
  uint32_t number;                   /**< 推荐车道信息编号, 离自车由近及远, 从1开始顺序编号 */
  uint32_t recom_lane[FU_RECOM_LANE_SIZE];  /**< 推荐车道编号序列 */
  int recom_lane_size;
  uint32_t distance;                   /**< 推荐车道离自车距离, cm*/
};

/**
 * @brief fusion map recom lane
 *  融合地图路口
 */
struct FusionMapRecomLane {
  RecomLane recom_lanes[FU_RECOM_LANES_SIZE];
  int recom_lanes_size;
};

/**
 * @brief remain lane dis
 *  路段
 */
struct DisEnd {
  uint32_t distance;                /**< 终点距离, 2km内若有routing终点 */
  ad_std::Point2d point;    /**< 终点坐标 */
};

struct FusionMap {
  ad_std::HeaderPOD header; /**< header */
  FusionMapRampJunction ramp_junction;
  FusionMapStopline stop_line;
  FusionMapTrafficLight map_trafficlight;
  FusionMapTransition transition;
  FusionMapCrosswalk cross_walk;
  LaneWidth lane_width;
  FusionMapWaitArea waitarea;
  FusionMapLaneInfoSection lane_section;
  FusionMapJunction junction;
  RemainLaneDis remain_lane_dis;
  FusionMapRecomLane recom_lanes;
  DisEnd dis_end;
};

struct BigFusion : public ad_std::MessageBase {
  ad_std::HeaderPOD header;
  FusionObjectsTab objects_tab;
  FusionLanes lanes;
  FusionTrafficLights traffic_lights;
  FusionTrafficSign traffic_signs;
  FusionParkSpace park_spaces;
  FusionFreeSpace free_spaces;
  FusionMap fusion_map;
  FusionStopLines stop_lines;
  FusionCrosswalks cross_walks;
};

} // end ad_world


#endif
