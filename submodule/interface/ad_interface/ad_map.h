/******************************************************************
Copyright(c) 2023-2025 ad All rights reserved.

author:ad_MAP
date:201923-09-21 10:00:00
filename:ad_map.h
contents:Unified definition of the map content message interface required by
each module of the intelligent driving system
******************************************************************/

#ifndef ad_INTERFACE_ad_MAP_INTERFACE_H
#define ad_INTERFACE_ad_MAP_INTERFACE_H

#include "stddef.h"
#include "stdint.h"

#include <algorithm>
#include <string>
#include <vector>

namespace ad_map {

#define INVALID_PATH_ID 0                          ///< 无效的路径ID
#define INVALID_OFFSET 0xFFFFFFFF                  ///< 无效的Offset
#define INVALID_TM_STAMP 0x7FFFFFFFFFFFFFFF        ///< 无效的时间戳
#define INVALID_TM_POSITION_AGE 0x7FFFFFFFFFFFFFFF ///< 无效的位置消息时间差
#define INVALID_LANE_IDX 0xFF                      ///< 无效的车道编号
#define INVALID_LONGITUDE 0x7FFFFFFF               ///< 无效的坐标经度
#define INVALID_LATITUDE 0x7FFFFFFF                ///< 无效的坐标纬度
#define INVALID_HEADING_DEGREE 361                 ///< 无效的航向角度
#define INVALID_CURVATURE 1023                     ///< 无效的曲率值
#define INVALID_SLOPE 10000                        ///< 无效的坡度值
#define INVALID_SPEED 10000                        ///< 无效的速度值
#define INVALID_ACC_SPEED 10000                    ///< 无效的加速度
#define INVALID_PROBABILITY -1.0f                  ///< 无效的置信度
#define INVALID_WIDTH 0x7FFFFFFF                   ///< 无效的宽度
#define INVALID_DISTANCE 0x7FFFFFFF                ///< 无效的距离
#define INVALID_LEVEL 0x7FFFFFFF                   ///< 无效层级
#define INVALID_MAP_AGE 0xFFFFFFFF                 ///< 无效的图龄
#define INVALID_VERSION 0xFFFFFFFF                 ///< 无效的版本

#define VALID_PATH_ID_MIN 8 ///< 有效的PathID起始值，一般为主路径ID
#define VALID_LANE_IDX_MIN 0       ///< 有效的车道编号
#define INVALID_SPEED_LIMIT 0x7FFF ///< 无效的限速值 #27620

/**
 * 处理结果
 */
enum class Av3hrCode : uint8_t {
  OK = 0,         ///< 处理成功
  FAIL,           ///< 处理失败
  NOT_SUPPORT,    ///< 不支持
  INTERNAL_ERROR, ///< 内部错误
  NO_INITIAL,     ///< 实体未初始化
  ARGUMENT_ERROR, ///< 参数错误
  NO_MANUAL_PLAN, ///< 无手动规划路径
  FRONT_NO_CROSS, ///< 前方直行指定范围内无路口
  STATIC_DATA_OK, ///< 静态数据生成成功(动态引导线生成失败，生成车道中心线数据)
  NO_POSITION,    ///< 没有收到位置信号
  NOT_MATCH, ///< 没有匹配成功,可能没有地图信息或者不在路网上
  DEVIATION_PATH, ///< 位置不在道路上,有一定的偏离
  NO_PLAN_DATA,   ///< 没有规划数据
};

/**
 * @brief 车道方向
 */
enum class ChangeMode : uint8_t {
  CREATE = 0, ///< 创建
  UPDATE = 1, ///< 更新
  DELETE = 2, ///< 删除
};

/**
 * @brief 属性类型
 */
enum class ProfileType : uint8_t {
  NODE = 0,                   ///< 路口（有道路合流/分歧的位置）
  HEADING_CHANGE = 2,         ///< 航向信息
  LANE_MODEL = 3,             ///< 车道模型，用于描述车道的属性
  LANE_CONNECTIVITY = 4,      ///< 车道拓扑连接
  LINEAR_OBJECTS = 5,         ///< 线性对象
  LANES_GEOMETRY = 6,         ///< 车道几何
  FUNCTIONAL_ROAD_CLASS = 13, ///< 道路功能等级
  FORM_OF_WAY = 15,           ///< 道路类型
  TUNNEL = 17,                ///< 隧道
  BRIDGE = 18,                ///< 桥梁
  CURVATURE = 20, ///< 曲率 511:表示直线 <511:表示左弯 >511:表示右弯 1023:无效
                  ///< 具体格式见：ADASIS V2 PROTOCOL 10.1章
  SLOPE = 21, ///< 坡度 [0,124]: 0% - 24.8% [125]: 25% - 30% [126]: >30%
              ///< 下坡为负、上坡为正
  TRAFFIC_SIGN = 25,             ///< 交通标记
  TRAFFIC_LIGHT = 26,            ///< 交通信号灯
  SPECIAL_SITUATION = 27,        ///< 特殊场景的数据
  EFFECTIVE_SPEED_LIMIT = 28,    ///< 限速
  PART_OF_CALCULATED_ROUTE = 35, ///< 是否属于规划的结果
  DRIVING_SIDE = 38,             ///< 【GLOBALDATA】驾驶方向信息
  VERSION_PROTOCOL =
      40, ///< 【GLOBALDATA】协议版本信息(2^24 * MAJOR + 2^16 * MINOR + SUB)
  MAP_AGE = 43, ///< 【GLOBALDATA】地图图龄信息(距离1970-01-01的小时数)
  MAP_STATUS = 45,                 ///< 【GLOBALDATA】地图状态信息
  ROAD_CLASS = 100,                ///< 道路等级
  EFFECTIVE_MIN_SPEED_LIMIT = 101, ///< 最低限速
  CROSS_SLOPE = 102,         ///< 横向坡度 [0,124]: 0% - 24.8% [125]: 25% -
                             ///< 30% [126]: >30% 左斜为负、右斜为正
  LINE_MARKING_TYPE = 103,   ///< 地面标线的类型（单双、虚实）
  LINE_MARKING_COLOUR = 104, ///< 地面标线的颜色
  LINE_MARKING_LOGIC = 105, ///< 地面标线的逻辑属性（真实边线、虚拟边线）
  ROAD_LEVEL = 106,              ///< 道路层级
  WHOLE_ROUTE = 107,             ///< 全局路径
  AUX_LANE_INFO = 109,           ///< 车道的扩展属性
  PASSABLE = 110,                ///< 可通行车道的信息
  SAME_DIRECTION_SECTIONS = 111, ///< 同向道路信息
  ROAD = 112,                    ///< 道路信息，无具体的属性
  ROAD_WIDTH = 114,              ///< 道路宽度
  LINE_MARKING_WIDTH = 115,      ///< 地面标线的宽度
  LINE_MARKING_SOURCE = 116, ///< 地面标线的来源(仅标线类型为道路外侧线时有效)
  NA = 255, ///< 无效
};

/**
 * @brief 行驶方向
 */
enum class LinkDirection : uint8_t {
  NO_DIRECTION = 0,       ///< 无
  POSITIVE_DIRECTION = 1, ///< 正向
  NEGATIVE_DIRECTION = 2, /// 反向
  BOTH_DIRECTION = 3,     /// 双向
};

/**
 * @brief 道路朝向
 */
enum class LinkOrientation : uint8_t {
  NONE = 0,       ///< 无
  NORTH = 1,      ///< NORTH
  EAST = 2,       ///< EAST
  SOUTH = 3,      ///< SOUTH
  WEST = 4,       ///< WEST
  NORTH_EAST = 5, ///< NORTH_EAST
  SOUTH_EAST = 6, ///< SOUTH_EAST
  SOUTH_WEST = 7, ///< SOUTH_WEST
  NORTH_WEST = 8, ///< NORTH_WEST
};

/**
 * @brief 车道方向
 */
enum class LaneDirection : uint8_t {
  NONE = 0,                      ///< 无
  BOTH = 1,                      ///< 双向
  ALONG_DRIVING_DIRECTION = 2,   ///< 与行驶方向相同
  AGAINST_DRIVING_DIRECTION = 3, ///< 与行驶方向相反
};

/**
 * @brief 过渡类型
 */
enum class LaneTransition : uint8_t {
  NONE = 0,      ///< 无
  OPENING = 1,   ///< OPENING
  CLOSING = 2,   ///< CLOSING
  MERGING = 3,   ///< 合并
  SPLITTING = 4, ///< 分支
};

/**
 * @brief 车道连接关系
 */
enum class LaneConnect : uint8_t {
  NONE = 0,        ///< 无
  SPLIT = 1,       ///< 分离
  MERGE = 2,       ///< 汇流
  CONTINUE = 3,    ///< 保持直行
  SPLIT_MERGE = 4, ///< 先分离再合流
  START = 5,       ///< 开始
  END = 6,         ///< 结束
  NA = 255,        ///< NA
};

/**
 * @brief 车道类型
 */
enum class LaneType : uint8_t {
  UNKNOWN = 0,                        ///< UNKNOWN
  NORMAL = 1,                         ///< 普通车道
  SERVICE_FORK = 4,                   ///< 分歧辅道
  SERVICE_CONFLUENCE = 5,             ///< 汇流辅道
  EXPAND = 6,                         ///< 扩充车道
  REDUCE = 7,                         ///< 缩减车道
  ETC = 8,                            ///< ETC 车道
  CHARGE = 9,                         ///< 收费通道
  EMERGENT = 10,                      ///< 紧急避险
  WARNING = 11,                       ///< 警示提醒车道
  BUS_ONLY = 12,                      ///< BUS 专用道
  EMERGENCY_DRIVEWAY = 14,            ///< 应急车道
  LOW_SPEED_CLIMBING = 15,            ///< 缓速爬坡车道
  INNER_BRANCH = 16,                  ///< 车道内分支
  WAITING_TURN = 17,                  ///< 待转区域
  U_TURN = 18,                        ///< U-TURN专用道
  R_TURN = 19,                        ///< 右转专用道
  NON_DRIVEWAY = 20,                  ///< 非机动车道
  PARKING = 21,                       ///< 停车道
  CONNECTION = 22,                    ///< 连接车道
  TIDAL = 23,                         ///< 潮汐车道
  VARIABLE_DIRECTION = 24,            ///< 可变导向车道
  HOV = 25,                           ///< HOV
  CASH_EXACT_CHANGE = 26,             ///< 人工收费车道
  ACCELERATION = 27,                  ///< 加速车道
  DECELERATION = 28,                  ///< 减速车道
  ACCELERATION_AND_DECELERATION = 29, ///< 减速减速共用车道
  ACCELERATION_OR_DECELERATION = 30,  ///< 加速或减速车道
  ETC_CHANGE_SIDE = 89,               ///< ETC与普通收费并行车道
  CARPOOL_ONLY = 99,                  ///< CARPOOL专用道
  NOT_INVESTIGATED = 100,             ///< 未调查
  NA = 127,                           ///< N/A
};

/**
 * @brief 车道转向类型
 */
enum class TurnType : uint8_t {
  GO_STRAIGHT = 0,                            // 直行
  TURN_RIGHT = 1,                             // 右转
  GO_STRAIGHT_OR_TURN_RIGHT = 2,              // 直行或右转
  TURN_LEFT = 3,                              // 左转
  GO_STRAIGHT_OR_TURN_LEFT = 4,               // 直行或左转
  TURN_LEFT_OR_TURN_RIGHT = 5,                // 左转或右转
  GO_STRAIGHT_OR_TURN_LEFT_OR_TURN_RIGHT = 6, // 直行或左转或右转
  U_TURN = 7,                                 // 掉头
  GO_STRAIGHT_OR_U_TURN = 8,                  // 直行或掉头
  TURN_LEFT_OR_U_TURN = 9                     // 左转或掉头
};

/**
 * @brief 车道边线类型
 */
enum class ProfileLineMarking : uint8_t {
  UNKNOWN = 0,                 ///< UNKNOWN
  NONE = 1,                    ///< 无边线
  SOLID_LINE = 2,              ///< 单实线
  DASHED_LINE = 3,             ///< 单虚线
  DOUBLE_SOLID_LINE = 4,       ///< 双实线
  DOUBLE_DASHED_LINE = 5,      ///< 双虚线
  LEFT_SOLID_RIGHT_DASHED = 6, ///< 双线（左实右虚）
  RIGHT_SOLID_LEFT_DASHED = 7, ///< 双线（左虚右实）
  DASHED_BLOCKS = 8,           ///<
  SHADED_AREA = 9,             ///< 阴影区
  PHYSICAL_DIVIDER = 10,       ///<
};

/**
 * @brief 车道边线颜色类型
 */
enum class ProfileLineMarkingColour : uint8_t {
  WHITE = 0,     ///< 白色
  LIGHTGRAY = 1, ///< 浅灰
  GRAY = 2,      ///< 灰色
  DARKGRAY = 3,  ///< 深灰
  BLACK = 4,     ///< 黑色
  RED = 5,       ///< 红色
  YELLOW = 6,    ///< 黄色
  GREEN = 7,     ///< 绿色
  CYAN = 8,      ///< 青色
  BLUE = 9,      ///< 蓝色
  ORANGE = 10,   ///< 橙色
  UNKNOWN = 15,  ///< 未调查
};

/**
 * @brief 车道边线逻辑类型
 */
enum class ProfileLineMarkingLogic : uint8_t {
  UNKNOWN = 0,              ///< UNKNOWN
  REAL = 1,                 ///< 真实边线
  DIVERSION_GUIDE = 2,      ///< 分歧路口导流线
  CONFLUENCE_GUIDE = 3,     ///< 汇流路口导流线
  LANE_INCREASE_GUIDE = 4,  ///< 车道数增加导流线
  LANE_DECREASE_GUIDE = 5,  ///< 车道数减少导流线
  LANE_MISSING_ZONE = 6,    ///< 车道线缺失区域
  SPLIT_OUT_OF_COLLECT = 7, ///< 收录范围外分割线
  LANE_BREAK_ZONE = 8,      ///< 车道线中断区
  LANE_UNRECOGNIZED = 9,    ///< 车道线无法识别
  LANE_CONN = 10,           ///< 连接车道边线
};

/**
 * @brief 车道边线线类型
 */
enum class ProfileCurveType : uint8_t {
  NOT_PRESENT = 0,   ///< NOT_PRESENT
  POLYLINE = 1,      ///< 折线
  BEZIER_SPLINE = 2, ///< 贝赛尔曲线
};

/**
 * @brief 道路类别
 */
enum class ProfileFormOfWay : uint8_t {
  NO_SPECIAL = 0,              ///< 未调查
  RAMP = 1,                    ///< 匝道
  ROUNDABOUT = 2,              ///< 环岛
  PARALLEL = 3,                ///< 平行于封闭道路
  SERVICE_ROAD = 4,            ///< 主辅连接路
  MAIN_ROAD = 5,               ///< 主路
  SQUARE = 6,                  ///< 未定义交通区域
  PEDESTRIAN_ZONE = 7,         ///< 园区内部路
  PEDESTRIAN = 8,              ///< 步行街
  ROUND_ABOUT_INTERIOR = 9,    ///< 环岛内连接路
  SLIP_ROAD = 10,              ///< 提前右转/左转/掉头专用道
  SPECIAL_TRAFFIC_FIGURE = 11, ///< 类似于环岛
  BOUNDARY = 12, ///< 区域边界线（比如国界线，省界线）
  UNKNOWN = 255, ///< 未调查
};

/**
 * @brief 道路类型的标志
 */
enum class ProfileRoadTypeFlag : uint8_t {
  CONTROL_ACCESS = 0,       ///< 封闭道路
  U_TURN = 1,               ///< 掉头路
  PARKING = 2,              ///< 园区内部路
  ELEVATED = 3,             ///< 高架
  UNDERPASS = 4,            ///< 地下通道
  UNDER_CONSTRUCTION = 5,   ///< 在建道路
  MOTORWAY = 6,             ///< 高速道路
  SERVICE_AREA = 7,         ///< 服务区道路
  TOLL = 8,                 ///< 收费站道路
  COMPLEX_INTERSECTION = 9, ///< 复杂交叉点
  PLURAL_JUNCTION = 10,     ///< 复合路口
  NA = 255,                 ///< 未调查
};

/**
 * @brief 道路种别
 */
enum class ProfileRoadClass : uint8_t {
  UNKNOWN = 0,           ///< 未调查
  URBAN_HIGHWAY = 1,     ///< 都市间高速
  URBAN_SEALED_ROAD = 2, ///< 都市内封闭路
  NATIONAL_ROAD = 3,     ///< 国道
  PROVINCIAL_ROAD = 4,   ///< 省道
  COUNTY_ROAD = 5,       ///< 县道
  ORDINARY_ROAD = 6,     ///< 一般道路
  NARROW_ROAD = 7,       ///< 细道路
};

/**
 * @brief 线性对象类型
 */
enum class LinearObjectType : uint8_t {
  CENTER_LINE = 0,  ///< 中心线
  LANE_MARKING = 1, ///< 边线
  GUARDRAIL = 2,    ///< 护栏
  CURB = 4,         ///< 路缘石
  WALL = 5,         ///< 墙
  OUTSIDE_LINE = 6, ///< 外侧线
  NA = 255,         ///< 无效
};

/**
 * @brief 特殊位置类型
 */
enum class SpecialSituationType : uint16_t {
  UNKNOWN = 0,               ///< 未知
  TOLLBOOTH = 250,           ///< 收费站
  PEDESTRIAN_CROSSING = 252, ///< 人行道
  SPEED_BUMP = 253,          ///< 减速带
  STOP_LINE = 261,           ///< 停止线
  GATE = 262,                ///< 闸机
  DESTINATION = 263,         ///< 目的地
};

enum class StopLineType : uint8_t {
  UNKNOWN = 0,          ///< 未知
  STOP_LINE = 1,        // 停止线
  YIELD_LINE = 2,       // 停车让行线
  SLOW_DOWN_LINE = 3,   // 减速让行线
  VIRTUAL_STOP_LINE = 4 // 虚拟停止线
};

/**
 * @brief 交通信号灯类型
 */
enum class TrafficLightType : uint8_t {
  INTERSECTION = 0,                        ///< 道口信号灯
  RAMP_METER = 1,                          ///< 匝道信号灯
  TOLLBOOTH = 2,                           ///< 收费站信号灯
  PEDESTRIAN_CROSSING_FOR_PEDESTRIANS = 3, ///< 人行横道行人型号等
  PEDESTRIAN_CROSSING_FOR_VEHICLES = 4,    ///< 人行横道机动车信号灯
  BICYCLE_CROSSING = 5,                    ///< 非机动车道信号灯
  TUNNEL = 6,                              ///< 隧道信号灯
  BRIDGE = 7,                              ///< 桥梁信号灯
  LANE_CONTROL = 8,                        ///< 车道管制信号灯
  TAIL_WAY = 9,                            ///< 铁路信号灯
  TRAM = 10,                               ///< 有轨电车信号灯
  DIRECTION_INDICATE = 11,                 ///< 方向指示信号灯
  FLASHING_WARNING = 12,                   ///< 闪光警告灯
  INVALID = 255, ///< 异常值 #29877 与FIDL文件保持一致
};

enum class TrafficLightConstructionType : uint8_t {
  HORIZONTAL = 0, ///< 水平
  VERTICAL = 1,   ///< 竖直
  DOGHOUSE = 2,
  UNKNOWN = 3, ///< 未调查
};

enum class LateralPosition : uint8_t {
  UNKNOWN = 0,
  RIGHT = 1,
  LEFT = 2,
  ABOVE = 4,
  SURFACE = 8,
};

/**
 * @brief 标志牌大类
 */
enum class TrafficSignType : uint8_t {
  DANGER = 0,                                   ///< 危险
  PASS_LEFT_OR_RIGHT_SIDE = 1,                  ///< 左右侧通过
  PASS_LEFT_SIDE = 2,                           ///< 左侧通过
  PASS_RIGHT_SIDE = 3,                          ///< 右侧通过
  DOMESTIC_ANIMALS_CROSSING = 4,                ///< 注意驯化动物穿行
  WILD_ANIMALS_CROSSING = 5,                    ///< 注意野生动物穿行
  ROADWORKS = 6,                                ///< 施工道路
  RESIDENTIAL_AREA = 7,                         ///< 住宅区域
  END_OF_RESIDENTIAL_AREA = 8,                  ///< 住宅区域结束
  RIGHT_BEND = 9,                               ///< 右转弯
  LEFT_BEND = 10,                               ///< 左转弯
  DOUBLE_BEND_RIGHT_FIRST = 13,                 ///< 双向转弯-右侧优先
  DOUBLE_BEND_LEFT_FIRST = 14,                  ///< 双向转弯-左侧优先
  CURVY_ROAD = 17,                              ///< 连续弯路
  OVERTAKING_BY_GOODS_VEHICLES_PROHIBITED = 20, ///< 禁止货运车辆超车
  END_OF_PROHIBITION_ON_OVERTAKING_FOR_GOODS_VEHICLES =
      21,                            ///< 解除货运车辆超车禁止
  DANGEROUS_INTERSECTION = 22,       ///< 危险交叉路口
  TUNNEL = 24,                       ///< 隧道
  FERRY_TERMINAL = 25,               ///< 轮渡码头
  NARROW_BRIDGE = 26,                ///< 窄桥
  HUMPBACK_BRIDGE = 28,              ///< 拱桥
  RIVERBANK = 29,                    ///< 河堤
  RIVERBANK_LEFT = 30,               ///< 左侧河堤
  LIGHT_SIGNALS = 31,                ///< 信号灯
  GIVE_WAY = 32,                     ///< 让行
  STOP = 33,                         ///< 停车
  PRIORITY_ROAD = 34,                ///< 优先行驶
  INTERSECTION = 35,                 ///< 交叉路口
  INTERSECTION_WITH_MINOR_ROAD = 36, ///< 支路优先路口
  INTERSECTION_WITH_PRIORITY_TO_THE_RIGHT = 37, ///< 右侧优先路口
  DIRECTION_TO_THE_RIGHT = 38,                  ///< 通向右方道路
  DIRECTION_TO_THE_LEFT = 39,                   ///< 通行左方道路
  CARRIAGE_WAY_NARROWS = 40,                    ///< 车道变窄
  CARRIAGE_WAY_NARROWS_RIGHT = 41,              ///< 右侧车道变窄
  CARRIAGE_WAY_NARROWS_LEFT = 42,               ///< 左侧车道变窄
  LANE_MERGE_LEFT = 43,                         ///< 左侧合流
  LANE_MERGE_RIGHT = 44,                        ///< 右侧合流
  LANE_MERGE_CENTER = 45,                       ///< 中间合流
  OVER_TAKING_PROHIBITED = 46,                  ///< 禁止超车
  END_OF_PROHIBITION_ON_OVERTAKING = 47,        ///< 解除超车禁止
  PROTECTED_PASSING_END = 48,                   ///< 解除通行受限
  PEDESTRIANS = 50,                             ///< 注意行人
  PEDESTRIAN_CROSSING = 51,                     ///< 人行横道
  CHILDREN = 52,                                ///< 注意儿童
  SCHOOL_ZONE = 53,                             ///< 学校区域
  CYCLISTS = 54,                                ///< 注意自行车
  TWO_WAY_TRAFFIC = 55,                         ///< 双向行驶
  RAILWAY_CROSSING_WITH_GATES = 56,             ///< 有人看守铁路道口
  RAILWAY_CROSSING_WITHOUT_GATES = 57,          ///< 无人看守铁路道口
  RAILWAY_CROSSING = 58,                        ///< 铁路通行
  TRAMWAY = 59,                                 ///< 电车轨道
  FALLING_ROCKS_LEFT = 61,                      ///< 左侧落石
  FALLING_ROCKS_RIGHT = 62,                     ///< 右侧落石
  STEEP_DROP_LEFT = 63,                         ///< 左侧急剧下降
  STEEP_DROP_RIGHT = 64,                        ///< 右侧急剧下降
  VARIABLE_SIGN_MECHANIC_ELEMENTS = 65,         ///< 可变交通标记
  SLIPPERY_ROAD = 66,                           ///< 易滑路段
  STEEP_ASCENT = 67,                            ///< 险升坡
  STEEP_DESCENT = 68,                           ///< 陡降坡
  UNEVEN_ROAD = 69,                             ///< 不平路面
  HUMP = 70,                                    ///< 驼峰路
  DIP = 71,                        ///< 低洼处（道路陡降然后上升）
  ROAD_FLOODS = 72,                ///< 洪水
  ICY_ROAD = 73,                   ///< 冰封道路
  SIDE_WINDS = 74,                 ///< 注意横风
  TRAFFIC_CONGESTION = 75,         ///< 交通拥挤
  HIGH_ACCIDENT_AREA = 76,         ///< 事故高发区域
  BEGINNING_OF_BUILT_UP_AREA = 77, ///< 建筑区域开始
  AUDIBLE_WARNING = 78,            ///< 鸣笛提醒
  END_OF_ALL_PROHIBITIONS = 79,    ///< 解除所有禁止
  VARIABLE_SIGN_LIGHT_ELEMENTS = 80,   ///< 可变信号灯
  PRIORITY_OVER_ONCOMING_TRAFFIC = 81, ///< 优先于对向道路
  PRIORITY_FOR_ONCOMING_TRAFFIC = 82,  ///< 对向优先
  END_OF_BUILT_UP_AREA = 83,           ///< 建筑区域结束
  SPEED_LIMIT = 87,                    ///< 限速
  END_OF_SPEED_LIMIT = 88,             ///< 解除限速
  SWING_BRIDGE = 89,                   ///< 平旋/转桥
  UNKNOWN = 255,                       ///< 未知
};

/**
 * @brief 转向信息
 */
enum class TurnInfo : uint8_t {
  UNKNOWN = 0,
  DIVERGENCE_STRAIGHT = 1,               ///< 分歧路口直行
  DIVERGENCE_RIGHT_FORWARD_TURN = 2,     ///< 分歧路口右前转
  DIVERGENCE_RIGHT = 3,                  ///< 分歧路口右转
  DIVERGENCE_RIGHT_BACKWARD_TURN = 4,    ///< 分歧路口右后转
  DIVERGENCE_LEFT_FORWARD_TURN = 5,      ///< 分歧路口左前转
  DIVERGENCE_LEFT = 6,                   ///< 分歧路口左转
  DIVERGENCE_LEFT_BACKWARD_TURN = 7,     ///< 分歧路口左后转
  DIVERGENCE_OFFSET_STRAIGHT = 8,        ///< 分歧路口错位直行
  DIVERGENCE_FORBID_TURING = 9,          ///< 分歧路口转向禁止
  CONFLUENCE_STRAIGHT = 10,              ///< 合流直行路口区间
  CONFLUENCE_RIGHT_FORWARD_TURN = 11,    ///< 合流路口右前转
  CONFLUENCE_RIGHT = 12,                 ///< 合流路口右转
  CONFLUENCE_RIGHT_BACKWARD_TURN = 13,   ///< 合流路口右后转
  CONFLUENCE_LEFT_FORWARD_TURN = 14,     ///< 合流路口左前转
  CONFLUENCE_LEFT = 15,                  ///< 合流路口左转
  CONFLUENCE_LEFT_BACKWARD_TURN = 16,    ///< 合流路口左后转
  CONFLUENCE_OFFSET_STRAIGHT = 17,       ///< 合流路口错位直行
  CONFLUENCE_FORBID_TURING = 18,         ///< 合流路口转向禁止
  INTERSECTION_STRAIGHT = 19,            ///< 交叉路口直行区间
  INTERSECTION_RIGHT_FORWARD_TURN = 20,  ///< 交叉路口右前转
  INTERSECTION_RIGHT = 21,               ///< 交叉路口右转
  INTERSECTION_RIGHT_BACKWARD_TURN = 22, ///< 交叉路口右后转
  INTERSECTION_LEFT_FORWARD_TURN = 23,   ///< 交叉路口左前转
  INTERSECTION_LEFT = 24,                ///< 交叉路口左转
  INTERSECTION_LEFT_BACKWARD_TURN = 25,  ///< 交叉路口左后转
  INTERSECTION_OFFSET_STRAIGHT = 26,     ///< 交叉路口错位直行
  INTERSECTION_FORBID_TURING = 27,       ///< 交叉路口转向禁止
  INTERSECTION_U_TURN = 28,              ///< 交叉路口掉头区间
  NA = 127,
};

/**
 * @brief 路径流向
 */
enum class PathType : uint8_t {
  MAIN = 0, ///< 主路径
  OUT = 1,  ///< 脱出子路径
  IN = 2,   ///< 汇入子路径
  NA = 3,   ///< 无效
};

/**
 * @brief 道路侧边
 */
enum class RoadSide : uint8_t {
  UNKNOWN = 0,
  LEFT = 1,
  RIGHT = 2,
  OVERHEAD = 3, ///< 上方/中间
  BOTH_DIRECTION = 4,
  SURFACE = 5,
  NA = 255,
};

/**
 * @brief 地图数据状态
 */
enum class ProfileMapStatus : uint8_t {
  MAP_NOT_AVAILABLE = 0, ///< 地图不可用
  MAP_LOADING = 1,       ///< 地图加载中
  MAP_AVAILABLE = 2,     ///< 地图可用
};

/**
 * @brief 驾驶规则
 */
enum class ProfileDrivingSide : uint8_t {
  RIGHT_HAND_DRIVING = 0, ///< 右手规则
  LEFT_HAND_DRIVING = 1,  ///< 左手规则
  UNKNOWN = 127           ///< 未知
};

/**
 * @brief 限速类型
 */
enum class SpeedLimitType : uint8_t {
  MAX = 0, ///< 最大限速类型
  MIN,     ///< 最小限速类型
  EXP,     ///< 经验速度类型
};

/**
 * @brief 连接匝道类型
 */
enum class ConnectRampType : uint8_t {
  IC_IN = 0, ///< 连接上匝道(驶入)
  IC_OUT,    ///< 连接下匝道(驶出)
  JC_IN,     ///< 高速间匝道(驶入)
  JC_OUT,    ///< 高速间匝道(驶出)
};

/**
 * @brief 导航点类型
 */
enum class NaviPointType : uint8_t {
  START = 0,   ///< 起点
  PASS_BY = 1, ///< 途经点
  DEST = 2,    ///< 目的点
};

enum class OriginSwitchReq : uint8_t {
  NONE = 0, ///< 无请求
  START,    ///< 进入切换模式请求
  EXIT,     ///< 结束切换模式请求
  GIVE_UP,  ///< 放弃切换模式请求
};

enum class LineMarkingSourceType : uint8_t {
  CURB = 0,                    ///< 路缘石
  GREENBELT = 1,               ///< 绿化带
  GUARDRAIL = 2,               ///< 护栏
  GROUND_MARKING = 3,          ///< 地面印刷线
  DIVERSION_ZONE = 4,          ///< 导流线
  SOURCE_VIRTUAL = 5,          ///< 虚拟
  WALL = 6,                    ///< 墙壁
  DITCH = 7,                   ///< 排水渠
  BARRIER_SOUND = 8,           ///< 隔音棚
  PLATFORM = 9,                ///< 道外水泥台
  ROAD_MATERIAL_BOUNDARY = 10, ///< 道路材质铺设边界
  FENCE = 11,                  ///< 栅栏
  DIKE_DAM = 12,               ///< 护堤
  COLUMN = 13,                 ///< 立柱
  OTHER = 15,                  ///< 其他
  UNKNOWN = 0XFF,              ///< 未知
};

/**
 * @brief 绝对坐标
 */
struct WGS84Point {
  int32_t longitude; ///< 经度，单位：0.0000001度
  int32_t latitude;  ///< 纬度，单位：0.0000001度
  int32_t altitude;  ///< 海拔，单位：cm

  WGS84Point() { clear(); }

  WGS84Point(const int32_t lon, const int32_t lat, const int32_t alt) {
    longitude = lon;
    latitude = lat;
    altitude = alt;
  }

  inline void clear() {
    longitude = INVALID_LONGITUDE;
    latitude = INVALID_LATITUDE;
    altitude = 0;
  }

  inline bool isValid() const {
    return (INVALID_LONGITUDE != longitude && INVALID_LATITUDE != latitude);
  }
};

struct UTMPoint {
  // 单位为cm
  int32_t E; ///< 东向
  int32_t N; ///< 北向
  int32_t U; ///< 天向

  UTMPoint() { clear(); }

  UTMPoint(const int32_t East, const int32_t North, const int32_t Up) {
    E = East;
    N = North;
    U = Up;
  }

  inline void clear() {
    E = -1;
    N = -1;
    U = 0;
  }

  inline bool isValid() const { return (E >= 0 && N >= 0); }
};

/**
 * @brief 车辆位置
 */
struct VehiclePosition : public WGS84Point {
  uint64_t timestamp; ///< 定位信号时间戳
  float heading; ///< 车辆航向角（正北为0，顺时增加），单位：度
  float speed; ///< 速度
};

/**
 * @brief 地图属性
 */
struct profilevalue {
  profilevalue() {}
  virtual ~profilevalue() {}

  virtual size_t bufSize() const { return 0; }
  virtual size_t serialize(char *buf) const { return 0; }
  virtual size_t deserialize(char *buf) { return 0; }
};

/**
 * @brief 车道信息
 */
struct LaneInfo {
  uint64_t link_id; // 道路id
  uint64_t lane_id; // 车道id，全局唯一
  uint8_t lane_number =
      INVALID_LANE_IDX; ///< 车道编号，从右往左，从0开始编号,默认0xFF为无效值

  LaneDirection direction;   ///< 车道方向
  LaneTransition transition; ///< 过渡类型
  LaneConnect connect;       ///< 连接关系
  TurnType turn;             ///< 转向类型
  uint32_t length;           // 车道长度

  uint64_t center_line;                   ///< 中心线ID
  std::vector<uint64_t> left_boundaries;  ///< 左边线ID列表
  std::vector<uint64_t> right_boundaries; ///< 右边线ID列表

  std::vector<LaneType> types; ///< 车道类型
  uint64_t opp_adj_lane_id;    // 对向相邻车道id
};

/**
 * @brief 车道信息列表
 */
struct LaneModel : public profilevalue {
  std::vector<LaneInfo> lane_infos; ///< 车道信息列表

  LaneModel &operator=(const LaneModel &other) {
    if (&other != this) {
      lane_infos = other.lane_infos;
    }
    return *this;
  }
};

/**
 * @brief 线性对象几何
 */
struct LineGeometry {
  uint64_t line_id;               ///< 线ID
  ProfileCurveType curve_type;    ///< 线类型
  std::vector<WGS84Point> points; ///< 形点列表
};

/**
 * @brief 线性对象几何列表
 */
struct LineGeometrys : public profilevalue {
  std::vector<LineGeometry> line_geometrys; ///< 线性对象几何列表

  LineGeometrys &operator=(const LineGeometrys &other) {
    if (&other != this) {
      line_geometrys = other.line_geometrys;
    }
    return *this;
  }
};

/**
 * @brief 线性对象
 */
struct LinearObject {
  uint64_t line_id;                    ///< 线ID
  LinearObjectType type;               ///< 线性对象类型
  ProfileLineMarking line_marking;     ///< 边线类型
  ProfileLineMarkingColour line_color; ///< 边线颜色类型
  RoadSide road_side;                  ///< 方位
};

/**
 * @brief 线性对象列表
 */
struct LinearObjects : public profilevalue {
  std::vector<LinearObject> objects; ///< 线性对象列表

  LinearObjects &operator=(const LinearObjects &other) {
    if (&other != this) {
      objects = other.objects;
    }
    return *this;
  }
};

/**
 * @brief 边线类型控制点
 */
struct TypeControlPos {
  uint32_t dist; ///< 距离车道边线起点的距离，单位：cm
  ProfileLineMarking marking; ///< 边线类型

  TypeControlPos() {}

  TypeControlPos(uint32_t d, ProfileLineMarking m) {
    dist = d;
    marking = m;
  }
};

/**
 * @brief 边线颜色控制点
 */
struct ColourControlPos {
  uint32_t dist; ///< 距离车道边线起点的距离，单位：cm
  ProfileLineMarkingColour colour; ///< 颜色类型

  ColourControlPos() {}

  ColourControlPos(uint32_t d, ProfileLineMarkingColour c) {
    dist = d;
    colour = c;
  }
};

/**
 * @brief 边线逻辑控制点
 */
struct LogicControlPos {
  uint32_t dist; ///< 距离车道边线起点的距离，单位：cm
  ProfileLineMarkingLogic logic; ///< 逻辑类型

  LogicControlPos() {}

  LogicControlPos(uint32_t d, ProfileLineMarkingLogic l) {
    dist = d;
    logic = l;
  }
};

/**
 * @brief 车道边线类型控制点
 */
struct LineMarkingType {
  uint64_t line_id;                          ///< 线ID
  std::vector<TypeControlPos> type_controls; ///< 类型控制点集合
};

/**
 * @brief 车道边线颜色控制点
 */
struct LineMarkingColour {
  uint64_t line_id;                              ///< 线ID
  std::vector<ColourControlPos> colour_controls; ///< 颜色控制点集合
};

/**
 * @brief 车道边线逻辑控制点
 */
struct LineMarkingLogic {
  uint64_t line_id;                            ///< 线ID
  std::vector<LogicControlPos> logic_controls; ///< 逻辑控制点集合
};

/**
 * @brief 车道边线类型控制点集合
 */
struct LineMarkingTypeList : public profilevalue {
  std::vector<LineMarkingType>
      line_marking_controls; ///< 车道边线类型控制点集合

  LineMarkingTypeList &operator=(const LineMarkingTypeList &other) {
    if (&other != this) {
      line_marking_controls = other.line_marking_controls;
    }
    return *this;
  }
};

/**
 * @brief 车道边线颜色控制点集合
 */
struct LineMarkingColourList : public profilevalue {
  std::vector<LineMarkingColour>
      line_marking_controls; ///< 车道边线颜色控制点集合

  LineMarkingColourList &operator=(const LineMarkingColourList &other) {
    if (&other != this) {
      line_marking_controls = other.line_marking_controls;
    }
    return *this;
  }
};

/**
 * @brief 车道边线逻辑控制点集合
 */
struct LineMarkingLogicList : public profilevalue {
  std::vector<LineMarkingLogic>
      line_marking_controls; ///< 车道边线逻辑控制点集合

  LineMarkingLogicList &operator=(const LineMarkingLogicList &other) {
    if (&other != this) {
      line_marking_controls = other.line_marking_controls;
    }
    return *this;
  }
};

/**
 * @brief 车道连接关系
 */
struct LaneConnectivityPair {
  uint8_t initial_lane_number = INVALID_LANE_IDX; ///< 当前的路径编号
  uint32_t initial_path = INVALID_PATH_ID;        ///< 当前的车道编号
  uint64_t initial_lane;                          ///< 当前的lane_id
  uint8_t new_lane_number = INVALID_LANE_IDX;     ///< NEXT的车道编号
  uint32_t new_path = INVALID_PATH_ID;            ///< NEXT的路径编号
  uint64_t new_lane;                              ///< NEXT的lane_id
};

/**
 * @brief 车道连接关系
 */
struct LaneConnectivity : public profilevalue {
  std::vector<LaneConnectivityPair> connectivity_pairs; ///< 车道连接关系列表

  LaneConnectivity &operator=(const LaneConnectivity &other) {
    if (&other != this) {
      connectivity_pairs = other.connectivity_pairs;
    }
    return *this;
  }
};

/**
 * @brief 限速速度
 */
struct Speed {
public:
  Speed() : value(INVALID_SPEED_LIMIT), type(SpeedLimitType::MAX) {} // #27620
  uint32_t value;      ///< 限速值（255表示不限制）
  SpeedLimitType type; ///< 速度单位
};

/**
 * @brief 车道限速控制点信息
 */
struct EffectiveSpeedLimit : public profilevalue {
  Speed value; ///< 限速速度

  EffectiveSpeedLimit &operator=(const EffectiveSpeedLimit &other) {
    if (&other != this) {
      value = other.value;
    }
    return *this;
  }
};

struct OffsetFloat {
  uint32_t offset; ///< 起始位置
  float value;     ///< ADSIS数值
};

/**
 * @brief Offset-Float控制点信息
 * TODO: 目前仅支持一个，后续可能需要支持多个
 */
struct OffsetFloatProfileValue : public profilevalue {
  std::vector<OffsetFloat>
      values; ///< 航向角列表（正北为0，顺时针增加，单位：度）

  OffsetFloatProfileValue &operator=(const OffsetFloatProfileValue &other) {
    if (&other != this) {
      values = other.values;
    }
    return *this;
  }
};

/**
 * @brief 车道曲率控制点信息
 */
struct Curvature : public profilevalue {
  std::vector<OffsetFloat>
      values; ///< 511:表示直线 <511:表示左弯 >511:表示右弯 1023:无效
              ///< 具体格式见：ADASIS v2 Protocol 10.1章

  Curvature() = default;
  Curvature &operator=(const Curvature &other) {
    if (&other != this) {
      values = other.values;
    }
    return *this;
  }
};

/**
 * @brief 车道坡度控制点信息
 */
struct Slope : public profilevalue {
  std::vector<OffsetFloat> values; ///<[0,124]: 0% - 24.8% [125]: 25% - 30%
                                   ///<[126]: >30% 下坡为负、上坡为正

  Slope() = default;
  Slope &operator=(const Slope &other) {
    if (&other != this) {
      values = other.values;
    }
    return *this;
  }
};

/**
 * @brief 车道横向坡度控制点信息
 */
struct CrossSlope : public profilevalue {
  std::vector<OffsetFloat> values; ///<[0,124]: 0% - 24.8% [125]: 25% - 30%
                                   ///<[126]: >30% 左斜为负、右斜为正

  CrossSlope() = default;
  CrossSlope &operator=(const CrossSlope &other) {
    if (&other != this) {
      values = other.values;
    }
    return *this;
  }
};

/**
 * @brief 车道宽度控制点信息
 */
struct LaneWidthProfileValue : public profilevalue {
  std::vector<OffsetFloat> values; ///< unit:cm

  LaneWidthProfileValue() = default;
  LaneWidthProfileValue &operator=(const LaneWidthProfileValue &other) {
    if (&other != this) {
      values = other.values;
    }
    return *this;
  }
};

struct RoadWidthPoint {
  uint32_t dist;
  uint32_t dist_to_left;
  uint32_t dist_to_right;
};

struct RoadWidthProfileValue : public profilevalue {
  std::vector<RoadWidthPoint> road_widths;

  RoadWidthProfileValue &operator=(const RoadWidthProfileValue &other) {
    if (&other != this) {
      road_widths = other.road_widths;
    }
    return *this;
  }
};

/**
 * @brief 道路种别
 */
struct RoadClassProfileValue : public profilevalue {
  ProfileRoadClass road_class; ///< 道路种别

  RoadClassProfileValue &operator=(const RoadClassProfileValue &other) {
    if (&other != this) {
      road_class = other.road_class;
    }
    return *this;
  }
};

/**
 * @brief Road
 */
struct RoadProfileValue : public profilevalue {};

/**
 * @brief 道路类别
 */
struct FormOfWay : public profilevalue {
  ProfileFormOfWay road_type;                       ///< 道路类别
  std::vector<ProfileRoadTypeFlag> road_type_flags; ///< 道路类型的标志

  FormOfWay &operator=(const FormOfWay &other) {
    if (&other != this) {
      road_type = other.road_type;
      road_type_flags = other.road_type_flags;
    }
    return *this;
  }

  inline bool hasRoadTypeFlag(ProfileRoadTypeFlag flag) const {
    return road_type_flags.end() !=
           std::find(road_type_flags.begin(), road_type_flags.end(), flag);
  }

  /**
   * @brief 是否高速与普通道路连接路(IC)
   * @return
   */
  inline bool IsICRamp() const {
    return ((ProfileFormOfWay::RAMP == road_type) &&
            (road_type_flags.end() ==
             std::find(road_type_flags.begin(), road_type_flags.end(),
                       ProfileRoadTypeFlag::MOTORWAY)));
  }

  /**
   * @brief 是否高速间连接路(JC)
   * @return
   */
  inline bool IsJCRamp() const {
    return ((ProfileFormOfWay::RAMP == road_type) &&
            (road_type_flags.end() !=
             std::find(road_type_flags.begin(), road_type_flags.end(),
                       ProfileRoadTypeFlag::MOTORWAY)) &&
            (road_type_flags.end() !=
             std::find(road_type_flags.begin(), road_type_flags.end(),
                       ProfileRoadTypeFlag::CONTROL_ACCESS)));
  }

  /**
   * @brief 是否路口
   * @return
   */
  inline bool IsCross() const {
    return ((ProfileFormOfWay::RAMP != road_type) &&
            (road_type_flags.end() !=
             std::find(road_type_flags.begin(), road_type_flags.end(),
                       ProfileRoadTypeFlag::COMPLEX_INTERSECTION)) &&
            (road_type_flags.end() !=
             std::find(road_type_flags.begin(), road_type_flags.end(),
                       ProfileRoadTypeFlag::PLURAL_JUNCTION)));
  }
};

/**
 * @brief 当前道路中属于道路计算结果的车道编号
 */
struct PartOfCalculatedRoute : public profilevalue {
  bool part_of_calculated_route;

  PartOfCalculatedRoute &operator=(const PartOfCalculatedRoute &other) {
    if (&other != this) {
      part_of_calculated_route = other.part_of_calculated_route;
    }
    return *this;
  }
};

/**
 * @brief 是否隧道
 */
struct Tunnel : public profilevalue {
  bool tunnel; ///< 是否隧道

  Tunnel &operator=(const Tunnel &other) {
    if (&other != this) {
      tunnel = other.tunnel;
    }
    return *this;
  }
};

/**
 * @brief 是否桥梁
 */
struct Bridge : public profilevalue {
  bool bridge; ///< 是否桥梁

  Bridge &operator=(const Bridge &other) {
    if (&other != this) {
      bridge = other.bridge;
    }
    return *this;
  }
};

/**
 * @brief 交通标志
 */
struct TrafficSign : public profilevalue {
  TrafficSignType type;          ///< 交通标志类型
  std::vector<WGS84Point> shape; ///< 形点列表
  std::vector<WGS84Point> bbox;  ///< 包围盒

  WGS84Point center_point;
  uint64_t link_id;

  TrafficSign &operator=(const TrafficSign &other) {
    if (&other != this) {
      type = other.type;
      shape = other.shape;
      bbox = other.bbox;
    }
    return *this;
  }
};
/**
 * @brief 交通信号灯
 */
struct TrafficLight : public profilevalue {
  uint64_t id;
  WGS84Point point;                               ///< 位置
  TrafficLightType type;                          ///< 信号灯类型
  LateralPosition lateral_position;               // 方位
  TrafficLightConstructionType construction_type; ///< 安装位置
  std::vector<WGS84Point> bbox;                   ///< 包围盒

  TurnType turn_type;
  std::vector<uint64_t> lane_ids;

  TrafficLight &operator=(const TrafficLight &other) {
    if (&other != this) {
      point = other.point;
      type = other.type;
      construction_type = other.construction_type;
      lateral_position = other.lateral_position;
      bbox = other.bbox;
    }
    return *this;
  }
};

/**
 * @brief 特殊位置
 */
struct SpecialSituation : public profilevalue {
  SpecialSituationType type; ///< 特殊位置类型，例如人行横道，收费站等
  std::vector<WGS84Point> shape; ///< 形状点串

  SpecialSituation &operator=(const SpecialSituation &other) {
    if (&other != this) {
      type = other.type;
      shape = other.shape;
    }
    return *this;
  }
};

/**
 * @brief 停止线
 */
struct StopLine : public profilevalue {
  StopLineType type; ///< 特殊位置类型，例如人行横道，收费站等
  std::vector<WGS84Point> shape; ///< 形状点串

  std::vector<uint64_t> traffic_light_ids; ///< 关联的交通灯id
  std::vector<uint64_t> lane_ids;          ///< 关联的车道id

  StopLine &operator=(const StopLine &other) {
    if (&other != this) {
      type = other.type;
      shape = other.shape;
      traffic_light_ids = other.traffic_light_ids;
      lane_ids = other.lane_ids;
    }
    return *this;
  }
};

/**
 * @brief 箭头
 */
struct ArrowMarking : public profilevalue {
  TurnType type; ///< 特殊位置类型，例如人行横道，收费站等
  std::vector<WGS84Point> bbox; ///< 形状点串

  ArrowMarking &operator=(const ArrowMarking &other) {
    if (&other != this) {
      type = other.type;
      bbox = other.bbox;
    }
    return *this;
  }
};

/**
 * @brief 路口中不同方向道路的转向信息
 */
struct NodeArm {
  uint64_t id;        ///< 路口id
  uint32_t sub_path;  ///< 子路径ID
  TurnInfo turn_info; ///< 路口转向信息
  float turn_angle;   ///< 路口转向角度[0,180] [degrees]
  PathType path_type; ///< 子路径类型
};

/**
 * @brief 路口中各个方向道路的转向信息集合
 */
struct Node : public profilevalue {
  std::vector<NodeArm> node_arms; ///< 转向信息集合

  Node &operator=(const Node &other) {
    if (&other != this) {
      node_arms = other.node_arms;
    }
    return *this;
  }
};

/********************** ProfileControl Message **********************/

/**
 * @brief ProfileControl信息
 */
struct ProfileControl {
  uint32_t path_id;
  uint32_t offset;
};

/********************** PathControl Message **********************/

/**
 * @brief PathControl信息
 */
struct PathControl {
  uint32_t id;
  uint32_t parent_id;
  uint32_t offset;
};

/********************** Global Message **********************/

/**
 * @brief Global信息
 */
struct Global {
  ProfileType profile_type;  ///< 属性类型
  profilevalue *profilvalue; ///< 属性值

  Global() { profilvalue = nullptr; }

  ~Global() {
    if (nullptr != profilvalue) {
      delete profilvalue;
      profilvalue = nullptr;
    }
  }
};

/**
 * @brief 协议版本号
 */
struct VersionProtocol : public profilevalue {
  uint32_t value; ///< 协议版本号，例如：3.1.1, 2^24 * major + 2^16 * minor +
                  ///< sub
};

/**
 * @brief 地图数据发布时间
 */
struct MapAge : public profilevalue {
  uint32_t value; ///< 图龄，从1970年1月1日开始的小时数
};

/**
 * @brief 地图数据状态
 */
struct MapStatus : public profilevalue {
  ProfileMapStatus status; ///< 地图数据状态
};

/********************** v3_PositionMessage  *********************/
/**
 * @brief 车辆当前所在置信度最高的车道及其左右车道的车道宽度
 */
struct LaneWidth {
  uint32_t left_lane_width; ///< 左侧车道宽度，单位：cm
  uint32_t lane_width; ///< 当前车道（置信度最高的车道）宽度，单位：cm
  uint32_t right_lane_width; ///< 右侧车道宽度，单位：cm

  LaneWidth() {
    left_lane_width = 0U;
    lane_width = 0U;
    right_lane_width = 0U;
  }
};

/**
 * @brief 车辆可能在的车道信息
 */
struct Position {
  uint32_t path_id;     ///< 车道所在路径ID
  uint32_t offset;      ///< 车辆实时相对Path起点的Offset，单位：cm
  uint8_t current_lane; ///< 车辆所在车道的编号，从右往左，从1开始
  float speed;          ///< 车速 单位：m/s
  float relative_heading; ///<@brief 车辆与车道的相对航向，单位：度，
                          ///< - vehicle heading minus road heading
                          ///< - clockwise positive
  float probability; ///< 概率/置信度 (0, 100]
  bool on_lane;      ///< 车辆是否在车道上

  LaneWidth
      lane_width; ///<@brief 实时车道宽度,包含车辆车道及其左右车道的车道宽度
  uint32_t dist_to_right_lane_line; ///< 车辆到当前车道右边线的距离
  uint32_t dist_to_left_lane_line;  ///< 车辆到当前车道左边线的距离

  Position() {
    path_id = INVALID_PATH_ID;
    offset = INVALID_OFFSET;
    current_lane = 0U;
    relative_heading = 0.0F;
    probability = 0.0F;
    on_lane = false;

    dist_to_right_lane_line = 0;
    dist_to_left_lane_line = 0;
  }
};

/**
 * @brief 描述车辆的位置及状态信息，用于Av3HR程序向Av3HP程序提供定位用信息。
 */
struct CarPosition {
  uint64_t timestamp; ///< 从定位系统获取到GPS坐标坐标的时间 (in the system time
                      ///<  of a possible vehicle time master)
  uint64_t position_age; ///< 获取到GPS位置到向车辆总线发送位置消息的时间差
  float heading; ///< 车辆航向角（正北为0，顺时增加），单位：度
  float speed;                     ///< 车辆速度，m/s
  WGS84Point crd;                  ///< 车辆位置坐标
  std::vector<Position> positions; ///< 车辆可能在的车道信息集合
  LaneWidth
      lane_width; ///<@brief
                  ///< 实时车道宽度,包含车辆当前所在置信度最高的车道及其左右车道的车道宽度
  float vel_north; ///< 北向车速 m/s
  float vel_east;  ///< 东向车速 m/s
  float vel_up;    ///< 垂直向车速 m/s
  float acc_x;     ///< x轴加速度 m/s²
  float acc_y;     ///< y轴加速度 m/s²
  float acc_z;     ///< z轴加速度 m/s²
  float pitch; ///< 俯仰，弧度，水平向前为0，向上为正，向下为负 范围[-π,π)
  float roll; ///< 横滚，弧度，垂直向上为0，向右为正，向左为负 范围[-π,π)
  float gyro_x; ///< x轴角加速度 rad/s²
  float gyro_y; ///< y轴角加速度 rad/s²
  float gyro_z; ///< z轴角加速度 rad/s²

  CarPosition() {
    timestamp = INVALID_TM_STAMP;
    position_age = INVALID_TM_POSITION_AGE;
    heading = 0.0F;

    acc_x = 0.0f;
    acc_y = 0.0f;
    acc_z = 0.0f;
    pitch = 0.0f;
    roll = 0.0f;
    gyro_y = 0.0f;
    gyro_x = 0.0f;
    gyro_z = 0.0f;
  }
};

/********************** v3_ProfileMessage *********************/
/**
 * @brief 属性信息
 */
struct Profile {
  uint32_t instance_id;
  ChangeMode change;
  uint32_t path_id;               ///< 路径ID
  std::vector<uint8_t> lane_idxs; ///< 关联的车道
  uint32_t offset;                ///< 与路段起点间的距离，单位：cm
  uint32_t end_offset;
  bool end_offset_final;
  ProfileType profile_type;  ///< 属性类型
  profilevalue *profilvalue; ///< 属性值

  Profile() {
    path_id = INVALID_PATH_ID;
    offset = INVALID_OFFSET;
    profilvalue = NULL;
  }
  ~Profile() {
    if (nullptr != profilvalue) {
      delete profilvalue;
      profilvalue = NULL;
    }
  }
};

struct RampJunction {
  ConnectRampType type; ///< 连接匝道类型
  uint32_t
      offset; ///< 连接匝道的路口开始处距起点所在RoadSection起始的距离 单位:cm
  uint32_t end_offset; ///< 连接匝道的路口结束处距起点所在RoadSection起始的距离
                       ///< 单位:cm
  uint32_t index; ///< 在m_vRampJunction中的索引,从0开始
};

struct NaviPoint {
  NaviPointType type; ///< 导航点类型
  uint32_t offset;    ///< 导航点offset
  WGS84Point point;   ///< 导航点坐标
  uint32_t index;     ///< 在m_vNaviPoint中的索引,从0开始

  uint32_t path_id; ///< 路径id
  uint8_t lane_idx; ///< 所在的车道序号
};

/**
 * @brief 全局路径信息
 */
struct WholeRoute : public profilevalue {
  std::vector<RampJunction>
      ramp_junctions; ///< 全局路径中子路径上有匝道的路口信息集合(不包含高速间的匝道)
  std::vector<NaviPoint> navi_points; ///< 全局路径中导航点的坐标列表
  uint32_t whole_route_length;        ///< 全局路径长度 单位:cm

  WholeRoute &operator=(const WholeRoute &other) {
    if (&other != this) {
      ramp_junctions = other.ramp_junctions;
      navi_points = other.navi_points;
      whole_route_length = other.whole_route_length;
    }
    return *this;
  }
};

/**
 * @brief 原点切换信息
 */
struct SwitchPoint {
  WGS84Point origin;     ///< 原点
  WGS84Point new_origin; ///< 新的原点
  bool switching;        ///< 是否处在切换模式下

  SwitchPoint() { clear(); }

  void clear() {
    origin.clear();
    new_origin.clear();
    switching = false;
  }
};

struct AbsoluteVehiclePosition : public profilevalue {
  VehiclePosition position;

  AbsoluteVehiclePosition &operator=(const AbsoluteVehiclePosition &other) {
    if (&other != this) {
      position = other.position;
    }
    return *this;
  }
};

/**
 * @brief 道路信息
 */
struct LinkInfo {
  uint64_t link_id;            ///< link_id
  LinkDirection direction;     ///< 道路方向
  LinkOrientation orientation; ///< 朝向类型
  uint32_t length;             ///< 道路长度
  uint8_t lane_count;          ///< 车道数量
  std::vector<uint64_t> recommend_lanes;
  std::vector<uint64_t> forbid_lanes;
  std::vector<uint64_t> rsu;
  std::string name;
  std::string instruction;
  uint64_t center_line;                   ///< 中心线ID
  std::vector<uint64_t> left_boundaries;  ///< 左边线ID列表
  std::vector<uint64_t> right_boundaries; ///< 右边线ID列表
};

/**
 * @brief 道路信息列表
 */
struct LinkModel : public profilevalue {
  std::vector<LinkInfo> link_infos; ///< 车道信息列表

  LinkModel &operator=(const LinkModel &other) {
    if (&other != this) {
      link_infos = other.link_infos;
    }
    return *this;
  }
};

/**
 * @brief 对向车道信息
 */
struct OppositeLaneInfo {
  uint64_t lane_id; // 车道id
  uint64_t link_id; // 道路id

  uint8_t lane_number =
      INVALID_LANE_IDX; ///< 车道编号，从右往左，从0开始编号,默认0xFF为无效值
  LaneDirection direction;   ///< 车道方向
  LaneTransition transition; ///< 过渡类型
  LaneConnect connect;       ///< 连接关系
  TurnType type;             ///< 连接关系
  uint32_t length;           // 车道长度

  uint64_t center_line;                   ///< 中心线ID
  std::vector<uint64_t> left_boundaries;  ///< 左边线ID列表
  std::vector<uint64_t> right_boundaries; ///< 右边线ID列表

  std::vector<LaneType> types; ///< 车道类型
  uint64_t opp_adj_lane_id;    // 对向相邻车道id
};

/**
 * @brief 对向车道信息列表
 */
struct OppositeLaneModel : public profilevalue {
  std::vector<OppositeLaneInfo> opp_lane_infos; ///< 车道信息列表

  OppositeLaneModel &operator=(const OppositeLaneModel &other) {
    if (&other != this) {
      opp_lane_infos = other.opp_lane_infos;
    }
    return *this;
  }

  /**
   * @brief 规划状态
   */
  enum class RoutingStatus : uint8_t {
    OK = 0,      ///< 成功
    FAILURE = 1, ///< 失败
  };

  enum class RoutingStrategy : uint8_t {
    LEAST_TIME = 0,     ///< 时间最短
    LEAST_FEE = 1,      ///< 费用最低
    LEAST_DISTANCE = 2, ///< 距离最短
    REAL_TRAFFIC = 3,   ///< 实时交通
  };

  /**
   * @brief 全局导航信息
   */
  struct GlobalRouting {
    uint64_t time_pub; ///< 发布时间戳
    WGS84Point start;  // 获取用户给定的起点位置坐标
    WGS84Point end;    // 获取用户给定的终点位置坐标
    std::vector<WGS84Point> waypoints; // 获取用户给定的途经点位置坐标
    RoutingStatus status;              // 全局规划是否成功
    RoutingStrategy strategy;          // 全局规划策略
    uint32_t id;                       // 路径id
    uint32_t distance;                 // 路径距离
    uint32_t duration;                 // 路径耗时
    WGS84Point route_start;    // 匹配至SD路网上的起点位置坐标
    WGS84Point route_end;      // 匹配至SD路网上的终点位置坐标
    WGS84Point route_waypoint; // 匹配至SD路网上的途经点位置坐标
    std::string route_start_name; // 匹配至SD路网上的起点名称/所在道路名称
    std::string route_end_name; // 匹配至SD路网上的终点名称/所在道路名称
    std::string route_waypoint_name; // 匹配至SD路网上的途径点名称/所在道路名称
    std::vector<uint64_t> link_seq; // 该路径上的linkID序列
  };
};
} // namespace ad_map

#endif // ad_INTERFACE_ad_MAP_INTERFACE_H
