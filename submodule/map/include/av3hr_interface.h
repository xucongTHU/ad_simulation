/******************************************************************
Copyright(c) 2023-2030 CAIC All rights reserved.
文件名称:av3hr_interface.h
简要描述:Adasis V3 HR 模块接口
******************************************************************/
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_set>
#include <set>

#include "common/av3_types.h"

#include "maplib/common_tool_temp.h"   //todo:delete after coodinate using UTM in whole project
#include "maplib/maplib_macro.h"

namespace caic_map {
namespace maplib {
/**
 * @brief 变道信息
 */
struct LaneChangeInfo
{
    uint32_t offset;          ///< 变道位置距离路径起始距离(即变道后车道所在RoadSectin的Offset)
    uint8_t  lane_idx;        ///< 变道前车道序号 从右往左,从1递增
    uint8_t  next_lane_idx;   ///< 变道后车道序号 从右往左,从1递增
};

enum BoundaryType
{
    // todo:后面考虑是否放进av3_types.h
    UNKNOWN                 = 0,     ///< UNKNOWN
    NONE                    = 1,     ///< 无边线->现实中无边线，即虚拟
    SOLID_LINE              = 2,     ///< 单实线
    DASHED_LINE             = 3,     ///< 单虚线
    DOUBLE_SOLID_LINE       = 4,     ///< 双实线
    DOUBLE_DASHED_LINE      = 5,     ///< 双虚线
    LEFT_SOLID_RIGHT_DASHED = 6,     ///< 双线（左实右虚）
    RIGHT_SOLID_LEFT_DASHED = 7,     ///< 双线（左虚右实）
    DASHED_BLOCKS           = 8,     ///<
    SHADED_AREA             = 9,     ///< 阴影区
    PHYSICAL_DIVIDER        = 10,    ///<
    CURB                    = 12,    ///< 路缘
    NA                      = 255,   ///< 无效
};

class Av3hrInterface
{
private:
    /**
     * @brief 将其构造和析构成为私有的, 禁止外部构造与析构
     */
    Av3hrInterface();

    ~Av3hrInterface();

private:
    static Av3hrInterface* av3hr_instance_;
    static std::mutex      mutex_;

public:
    /**
     * @brief 静态方法,获取实例接口
     * @param  [in] config_str 配置参数，如是否生成子路径，路径删除距离阈值等，格式为Json
     * @return Av3hr* 指针指向的实例
     */
    static Av3hrInterface* GetInstance(const std::string config_str = "");

    /**
     * @brief 释放单实例，进程退出时调用
     */
    static void destroyInstance();

    // TODO: 函数定义参数，遵循先入参后出参的原则，对于入参需要增加const修饰，对于出参非特殊情况均采用指针形式
public:

    /**
     * @brief 获取位置信息
     * @param[out] position  位置信息
     * @return Av3hrCode
     * -  0 获取位置信息成功\n
     * -非0 获取位置信息失败
     */
    Av3hrCode GetPosition(PositionX& position);
    // 获取自车位置信息

    /**
     * @brief 获取路径上一定范围内指定地图属性信息
     * @param[out] profiles    属性信息
     * @param[out] timestamp    数据时间戳
     * @param[in]  path_id       路径ID,为0时表示获取所有path
     * @param[in]  start_offset  路径上范围的起点, 单位: cm
     * @param[in]  end_offset    路径上范围的终点, 单位: cm
     * @param[in]  profile_types   属性类别
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode GetProfile(std::map<ProfileType, std::vector<Profile*>>& profiles, uint64_t& timestamp,
                         const uint32_t path_id = 0, const uint32_t start_offset = 0,
                         const uint32_t                  end_offset    = INVALID_OFFSET,
                         const std::vector<ProfileType>& profile_types = std::vector<ProfileType>());

    /**
     * @brief 获取全局信息
     * @param[out] globals 属性信息(协议版本,地图状态,[图龄,交通规则行驶方向])
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode GetGlobal(std::vector<Global*>& globals);

    /**
     * @brief 获取切换原点信息
     * @param[out] sp 切换原点信息
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode GetSwitchOriginPoint(SwitchPoint& sp);

    /**
     * @brief 发送切换请求
     * @param[in] req 切换请求
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode SendSwitchReq(OriginSwitchReq req);

    /**
     * @brief 查询主路径
     * @param[out] path_id 主路径id
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode GetMainPath(uint32_t& path_id);

    /**
     * @brief 查询道路模型
     * @param[out] road_models    道路模型
     * @param[in]  path_id       路径ID
     * @param[in]  start_offset  路径上范围的起点, 单位: cm
     * @param[in]  end_offset    路径上范围的终点, 单位: cm
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode GetRoadModels(std::vector<RoadModel>& road_models, const uint32_t& path_id, const uint32_t& start_offset,
                            const uint32_t& end_offset);

    /**
     * @brief 查询车道模型
     * @param[out] lane_groups    车道模型
     * @param[in]  path_id       路径ID
     * @param[in]  start_offset  路径上范围的起点, 单位: cm
     * @param[in]  end_offset    路径上范围的终点, 单位: cm
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode GetLaneGroups(std::vector<LaneGroup>& lane_groups, const uint32_t& path_id, const uint32_t& start_offset,
                            const uint32_t& end_offset);
#if 0
    /**
     * @brief 查询车道模型
     * @param[out] lane_model    车道模型
     * @param[in]  path_id       路径ID
     * @param[in]  offset  路径上范围的起点, 单位: cm
     * @param[in]  end_offset    路径上范围的终点, 单位: cm
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode GetLaneModel(vector<caic_map::LaneModel> & lane_model, const uint32_t & path_id,
                           const uint32_t & offset, const uint32_t & end_offset);
#endif
    /**
     * @brief 查询地物对象
     * @param[out] object_attrs    对象模型
     * @param[in]  path_id       路径ID
     * @param[in]  start_offset  路径上范围的起点, 单位: cm
     * @param[in]  end_offset    路径上范围的终点, 单位: cm
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode GetObjects(std::vector<ObjectAttr>& object_attrs, const uint32_t path_id, const uint32_t start_offset,
                         const uint32_t end_offset);

    /**
     * @brief 获取子路径信息
     * @param[out] stub_infos    子路径列表
     * @param[in]  path_id       父路径ID
     * @param[in]  start_offset  路径上范围的起点, 单位: cm
     * @param[in]  end_offset    路径上范围的终点,0代表全部 单位: cm
     * @return Av3hrCode
     * -  0 获取属性信息成功\n
     * -非0 获取属性信息失败
     */
    Av3hrCode GetSubPaths(std::vector<Stub>& stub_infos, const uint32_t& path_id, const uint32_t& start_offset,
                          const uint32_t& end_offset);

    // 消息处理接口
    void SaveRecvData(const char* szRecvBuf, size_t iRecvSize);

    // 消息处理接口
    Av3hrCode ParseMessage(const uint8_t* message_buff, int size);

    /**
     * @brief 获取前方路口的道路模型数据
     * @param[out] intersection_roadModels_ranges 输出参数，前方前进方向的roadmodel的范围
     * @param[in] intersection_road_models 输入参数，前方路口
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetIntersectionRoadModelsRange(std::map<uint32_t, std::vector<uint32_t>>& intersection_roadModels_range,
                                             const std::vector<RoadModel>&              intersection_road_models);

    // 获取前方路口的道路模型距离数据
    /**
     * @brief 获取前方路口的道路模型数据
     * @param[out] intersection_roadModels_offset 输出参数，前方前进方向的roadmodel的范围
     * @param[in] intersection_road_models 输入参数，前方路口
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetIntersectionRoadModelsOffset(std::vector<uint32_t>& intersection_roadModels_offset,
                                              const std::vector<RoadModel>& intersection_road_models);

    // 获取前方路口的道路模型转弯信息
    /**
     * @brief 获取前方路口的道路模型数据
     * @param[out] intersection_roadModels_turn_type 输出参数，前方前进方向的roadmodel的范围
     * @param[in] intersection_road_models 输入参数，前方路口
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetIntersectionRoadModelsTurnType(std::vector<TurnInfo>& intersection_roadModels_turn_type,
                                                const std::vector<RoadModel>& intersection_road_models);

    /**
     * @brief 获取前方路口的道路模型数据
     * @param[out] intersection_road_models 输出参数，前方路口的道路模型
     * 其中offset和end_offset表示该roadModel的起始位置；turn_type表示该roadModel的转弯类型；
     * offset表示路口距离
     * @param[in] path_id 输入参数，path的id
     * @param[in] start_offset 输入参数，查找的开始偏置
     * @param[in] end_offset 输入参数，查找的结束偏置
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetIntersectionRoadModels(std::vector<RoadModel>& intersection_road_models, const uint32_t& path_id,
                                        const uint32_t& start_offset, const uint32_t& end_offset);
    /**
     * @brief 获取车辆当前位置的交换区域
     * @param[out] transition_lane_models 输出参数，当前位置到当前车道
     * @param[in] path_id 输入参数，path的id
     * @param[in] start_offset 输入参数，查找的开始偏置
     * @param[in] end_offset 输入参数，查找的结束偏置
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetTransitionLaneModels(std::vector<LaneGroup>& transition_lane_models, const uint32_t& path_id,
                                      const uint32_t& start_offset, const uint32_t& end_offset);

    /**
     * @brief 获取当前位置到当前车道的结尾的距离
     * @param[out] dist_to_current_lane_end 输出参数，当前位置到当前车道
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetDistToCurrentLaneEnd(int32_t& dist_to_current_lane_end);

    /**
     * @brief 获取车辆当前位置的交换区域
     * @param[out] dist_to_lane_end 输出参数，车道组结尾剩余距离
     * @param[in] path_id 输入参数，path的id
     * @param[in] start_offset 输入参数，查找的开始偏置
     * @param[in] end_offset 输入参数，查找的结束偏置
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetDistToLaneEndOfPose(int32_t& dist_to_lane_end,const uint32_t& path_id, 
                                    const uint32_t& start_offset, const uint32_t& end_offset);

    /**
     * @brief 获取当前位置到特定车道的结尾的距离
     * @param[out] dist_to_current_lane_end 输出参数，当前位置到当前车道
     * @param[in] laneModel 输入参数，车道组
     * @param[in] here_offset 输入参数，当前位置偏置
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetDistToLaneEndInLaneModel(int32_t& dist_to_current_lane_end, const LaneGroup& laneModel,
                                          const uint32_t& here_offset);

    /**
     * @brief 获取当前位置前方range_length距离的主路经的roadModel的推荐车道
     * @param[out] recommend_lanes 输出参数，前方range_length距离的主路经的roadModel的推荐车道
     * @param[in] range_length 输入参数，前方的距离
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetCurrentLaneRecommendLanes(std::map<uint32_t, std::vector<uint64_t>>& recommend_lanes,
                                           const uint32_t&                            range_length);

    /**
     * @brief 获取当前位置前方的主路经的roadModel的推荐车道
     * @param[out] recommend_lanes 输出参数，主路经的roadModel的推荐车道
     * @param[in] path_id 输入参数，主路经id
     * @param[in] start_offset,输入参数，起点位置
     * @param[in] end_offset 输入参数，终点位置
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetRecommendLanes2(std::map<uint32_t, std::vector<uint64_t>>& recommend_lanes, const uint32_t& path_id,
                                const uint32_t& start_offset, const uint32_t& end_offset);
    
    /**
     * @brief 获取当前位置前方的主路经的roadModel的推荐车道
     * @param[out] recommend_lanes 输出参数，主路经的roadModel的推荐车道
     * @param[in] path_id 输入参数，主路经id
     * @param[in] start_offset,输入参数，起点位置
     * @param[in] end_offset 输入参数，终点位置
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetRecommendLanes(std::map<uint32_t, std::unordered_set<uint8_t>>& recommend_lanes, const uint32_t& path_id,
                                const uint32_t& start_offset, const uint32_t& end_offset);

    /**
     * @brief 获取当前位置前方的主路经的roadModel的推荐车道
     * @param[out] recommend_lanes 输出参数，主路经的roadModel的推荐车道
     * @param[in] laneModels 输入参数，车道组数据
     * @param[in] path_id 输入参数，主路经id
     * @param[in] start_offset,输入参数，起点位置
     * @param[in] end_offset 输入参数，终点位置
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetRecommendLanesInLaneModels(std::map<uint32_t, std::unordered_set<uint8_t>>& recommend_lanes,
                                            const std::vector<LaneGroup> & laneModels, const uint32_t& path_id) ;

    /**
     * @brief 获取当前位置前方的主路经的roadModel的推荐车道
     * @param[out] recommend_lanes 输出参数，主路经的roadModel的推荐车道
     * @param[in] road_model 输入参数，道路模型
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetRecommendLanesInRoadModel(std::pair<uint32_t, std::vector<uint64_t>>& recommend_lanes,
                                           const RoadModel&                            road_model);
    /**
     * @brief 获取当前车道的车道数
     * @param[out] current_roadModel_lanes_count 输出参数，当前道路的车道数量
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetCurrentRoadModelLanesCount(uint8_t& current_roadModel_lanes_count);

    // 获取某定位位置所处道路的车道数(范例)
    /**
     * @brief 获取某定位位置所处道路的车道数(范例)
     * @param[out] current_roadModel_lanes_count 输出参数，某定位位置所处道路的车道数
     * @param[in] path_id 输入参数，主路经id
     * @param[in] start_offset,输入参数，起点位置
     * @param[in] end_offset 输入参数，终点位置
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetRoadModelLanesCountOfPose(uint8_t& current_roadModel_lanes_count,const uint32_t & path_id, 
                                           const uint32_t & start_offset, const uint32_t & end_offset);

    /**
     * @brief 获取某车道组的车道数量
     * @param[out] current_roadModel_lanes_count 输出参数，当前道路的车道数量
     * @param[in] lane_model 输入参数，车道组
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetRoadModelLanesCountInLaneModel(uint8_t& current_roadModel_lanes_count, const LaneGroup& lane_model);

    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] turn_type 输出参数，车道转弯类型
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetCurrentLaneTurnType(TurnType& turn_type);

    // 获取定位处车道转弯类型
    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] lane_turn_type 输出参数，车道的转弯类型
     * @param[in] path_id 主路径id,
     * @param[in] start_offset 查找的开始位置
     * @param[in] end_offset 查找的结束位置
     * @param[in] current_lane_id 当前车道线的id
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    
    Av3hrCode GetLaneTurnTypeOfPose(TurnType& lane_turn_type,const uint32_t & path_id, 
                                                const uint32_t & start_offset,const uint32_t & end_offset,
                                                const uint8_t & current_lane_id);

    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] turn_type 输出参数，车道的转弯类型
     * @param[in] laneModel 某一个具体的车道组
     * @param[in] lane_id 某一个车道的id
     * @param[in] position_offset 定位出的偏置
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetLaneTurnTypeInLaneModel(TurnType& turn_type, LaneGroup& laneModel, const uint8_t& lane_id,
                                         const uint32_t& position_offset);

    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] speed_limit 输出参数，限速值. 如果值为0,表示车辆位置无限速信息
     * @param[in] limit_type 最大限速或最低限速或经验限速
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetSpeedLimit(Speed& speed_limit, SpeedLimitType limit_type);

    // 获取车道限速(范例)
    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] speed_limit 输出参数，限速值. 如果值为0,表示车辆位置无限速信息
     * @param[in] limit_type 最大限速或最低限速或经验限速
     * @param[in] path_id 主路径id,
     * @param[in] start_offset 查找的开始位置
     * @param[in] end_offset 查找的结束位置
     * @param[in] current_lane_id 当前车道线的id
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetSpeedLimitOfPose(Speed& speed_limit, SpeedLimitType limit_type,
                                  const uint32_t & path_id, const uint32_t & start_offset,
                                  const uint32_t & end_offset, const uint8_t & current_lane_id );
    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] speed_limit 输出参数，限速值. 如果值为0,表示车辆位置无限速信息
     * @param[in] limit_type 最大限速或最低限速或经验限速
     * @param[in] laneModels 车道组模型
     * @param[in] lane_id 车道id
     * @param[in] position_offset 定位点
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetSpeedLimitInLaneModel(Speed& speed_limit, SpeedLimitType& limit_type, const LaneGroup& laneModel,
                                       const uint8_t& lane_id, const uint32_t& position_offset);

    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] lane_width 输出参数，车道宽度值. 当前和左右车道宽度
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetCurrentLaneWidth(LaneWidth& lane_width);

    // 获取车道宽度，左中右3车道宽度(范例)
    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] lane_width 输出参数，车道宽度值. 当前和左右车道宽度
     * @param[in] path_id 输入参数，主路径id
     * @param[in] start_offset 查找的开始位置，
     * @param[in] end_offset 查找的结束位置
     * @param[in] current_lane_id 当前车道id
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetLaneWidthOfPose(LaneWidth& lane_width, const uint32_t & path_id,
                                 const uint32_t & start_offset, const uint32_t & end_offset,
                                 const uint8_t & current_lane_id);

    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] lane_width 输出参数，车道宽度值. 当前和左右3个车道宽度
     * @param[in] laneModels 车道组模型
     * @param[in] path_id 道路id
     * @param[in] lane_id 车道id
     * @param[in] position_offset 定位点
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetLaneWidthInLaneModel(LaneWidth& lane_width, const LaneGroup& laneModels,
                                             const uint32_t& path_id, const uint8_t& lane_id,
                                             const uint32_t& position_offset);

    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] lane_start_end_point_info 输出参数，本车道起始点坐标.
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetCurrentLaneStartEndPointInfo(std::vector<WGS84Point>& lane_start_end_point_info);

    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] lane_start_end_point_info 输出参数，车道起点终点坐标
     * @param[in] path_id 道路id
     * @param[in] start_offset 查找的开始位置，
     * @param[in] end_offset 查找的结束位置
     * @param[in] current_lane_id 当前车道id
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetLaneStartEndPointInfoOfPose(std::vector<WGS84Point>& lane_start_end_point_info,
                                             const uint32_t & path_id, const uint32_t & start_offset,
                                             const uint32_t & end_offset, const uint8_t & current_lane_id);

    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] lane_start_end_point_infos 输出参数，车道起点终点信息
     * @param[in] laneModels 车道组模型
     * @param[in] lane_id 车道id
     * @param[in] position_offset 定位点
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetLaneStartEndPointInfoInLaneModel(
        std::map<uint8_t, std::vector<WGS84Point>>& lane_start_end_point_infos, const LaneGroup& laneModel,
        const std::vector<uint8_t>& lane_id);

    /**
     * @brief 获取车辆当前位置的限速
     * @param[out] lane_start_end_point_infos 输出参数，车道起点终点信息
     * @param[in] laneModels 车道组模型
     * @param[in] lane_id 车道id
     * @param[in] position_offset 定位点
     * @return Av3hrCode
     * -   0 成功\n
     * - 非0 失败
     */
    Av3hrCode GetLanesStartEndPointInfo(
        std::map<uint8_t, std::map<uint8_t, std::vector<WGS84Point>>>& lanes_start_end_point_infos,
        std::vector<LaneGroup>& laneModels, const std::vector<uint8_t>& lane_id, const uint32_t& position_offset);

    /**
     * @brief 获取前方一定范围内的推荐车道变道信息
     * @param[out] lane_change_info 输出参数, 前方一定范围内推荐车道变道信息
     * @param[in]  search_dist 输入参数, 向前方搜索的距离,单位:米,默认为UINT32_MAX,表示不限制
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方无路口\n
     * - 其他 处理失败
     */
    Av3hrCode GetRecommendLaneChangeInfo(std::vector<LaneChangeInfo>& lane_change_info,
                                         uint32_t                     search_dist = UINT32_MAX);

    /**
     * @brief 获取车辆前后方一定距离的规划推荐的车道路径形点串
     * @param[out] way_points 规划的推荐车道路径形点串
     * @param[in]  back_dist 车辆后方距离, 默认是最大值，0表示不输出
     * @param[in]  front_dist 车辆前方距离, 默认是最大值，0表示不输出
     * @return Av3hrCode
     * -  0 设置成功\n
     * -非0 设置失败
     */
    Av3hrCode GetRoutePlanWayPoints(std::vector<WGS84Point>& way_points, uint32_t back_dist = UINT32_MAX,
                                    uint32_t front_dist = UINT32_MAX);


    /**
     * @brief 获取车辆前方规划路径上的路口
     * @param[out] node_list 前方路口的信息
     * @param[in]  front_dist 车辆前方距离, 默认是最大值，0表示不输出
     * @param[in]  back_dist 车辆后方距离, 默认是0表示不输出
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方无路口\n
     * - 其他 处理失败
     */
    Av3hrCode GetIntersectionAhead(std::vector<Node>& node_list, uint32_t front_dist = UINT32_MAX,
                                   uint32_t back_dist = 0);

    /**
     * @brief 获取车辆前方规划路径上的匝道
     * @param[out] ramp_junctions 前方匝道的信息
     * @param[in]  front_dist 车辆前方距离, 默认是最大值，0表示不输出
     * @param[in]  back_dist 车辆后方距离, 默认是0表示不输出
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方无匝道\n
     * - 其他 处理失败
     */
    Av3hrCode GetRampJunctionsAhead(std::vector<RampJunction>& ramp_junctions, uint32_t front_dist = UINT32_MAX,
                                    uint32_t back_dist = 0);

    // 前方匝道
    /**
     * @brief 获取车辆前方规划路径上的匝道
     * @param[out] ramp_junctions 前方匝道的信息
     * @param[in]  path_id 输入参数，主路径的参数
     * @param[in]  start_offset 输入参数，查找的开始位置
     * @param[in]  end_offset 输入参数，查找的结束位置
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方无匝道\n
     * - 其他 处理失败
     */
    Av3hrCode GetRampJunctions(std::vector<RampJunction>& ramp_junctions, const uint32_t & path_id,
                               const uint32_t & start_offset, const uint32_t & end_offset);

    /**
     * @brief 获取车辆前方规划路径上的待转区信息
     * @param[out] waiting_turns 前方待转区的信息
     * @param[in]  front_dist 车辆前方距离, 默认是最大值，0表示不输出
     * @param[in]  back_dist 车辆后方距离, 默认是0表示不输出
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方无待转区\n
     * - 其他 处理失败
     */
    Av3hrCode GetWaitingTurnAhead(std::vector<LaneGroup>& waiting_turns, uint32_t front_dist = UINT32_MAX,
                                  uint32_t back_dist = 0);

    
    // 前方待转区
    /**
     * @brief 获取车辆前方规划路径上的待转区信息
     * @param[out] waiting_turns 前方待转区的信息
     * @param[in] path_id 输入参数，主路径id
     * @param[in]  start_offset 输入参数，查找的开始位置
     * @param[in]  end_offset 输入参数，查找的结束位置
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方无待转区\n
     * - 其他 处理失败
     */
    Av3hrCode GetWaitingTurns(std::vector<LaneGroup>& waiting_turns, const uint32_t & path_id,
                                           const uint32_t & start_offset, const uint32_t & end_offset);

    /**
     * @brief 获取车辆前方规划路径上的人行道信息
     * @param[out] cross_walks 前方人行道的信息
     * @param[in]  front_dist 车辆前方距离, 默认是最大值，0表示不输出
     * @param[in]  back_dist 车辆后方距离, 默认是0表示不输出
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方人行道\n
     * - 其他 处理失败
     */
    std::vector<std::shared_ptr<caic_map::Profile>> GetCrossWalkAhead(uint32_t front_dist = UINT32_MAX,
                                                                      uint32_t back_dist  = 0);

    /**
     * @brief 获取车辆前方规划路径上的人行道信息
     * @param[out] cross_walks 前方人行道的信息
     * @param[in] path_id 输入参数，主路经的id
     * @param[in]  start_offset 输入参数，查询的开始位置
     * @param[in]  end_offset 输入参数，查询的结束位置
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方人行道\n
     * - 其他 处理失败
     */
    Av3hrCode GetCrossWalks(std::vector<std::shared_ptr<caic_map::Profile>>& cross_walks, const uint32_t& path_id,
                            const uint32_t& start_offset, const uint32_t& end_offset);

    /**
     * @brief 获取车辆前方规划路径上的人行道的距离
     * @param[out] cross_walks_offsets 前方人行道位置信息
     * @param[in] cross_walks 前方人行道信息
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方人行道\n
     * - 其他 处理失败
     */
    Av3hrCode GetCrossWalksOffsets(std::vector<uint32_t>&                cross_walks_offsets,
                                   const std::vector<caic_map::Profile>& cross_walks);

    /**
     * @brief 获取车辆前方规划路径上的人行横道上的区域信息
     * @param[out] cross_walks_shapes 前方人行道区域信息
     * @param[in] cross_walks 前方人行道信息
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方人行道\n
     * - 其他 处理失败
     */
    Av3hrCode GetCrossWalksShapes(std::vector<std::vector<WGS84Point>>& cross_walks_shapes,
                                  const std::vector<caic_map::Profile>& cross_walks);

    /**
     * @brief 获取车辆前方规划路径上的停止线信息
     * @param[out] stop_lines 前方停止线的信息
     * @param[in]  front_dist 车辆前方距离, 默认是最大值，0表示不输出
     * @param[in]  back_dist 车辆后方距离, 默认是0表示不输出
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方无停止线\n
     * - 其他 处理失败
     */
    Av3hrCode GetStopLineAhead(std::vector<StopLine>& stop_lines, uint32_t front_dist = UINT32_MAX,
                               uint32_t back_dist = 0);

    /**
     * @brief 获取车辆前方规划路径上的停止线信息
     * @param[out] stop_lines 前方停止线的信息
     * @param[in] path_id 输入参数，主路经id
     * @param[in]  start_offset 输入参数, 查询的开始位置，
     * @param[in]  end_offset 输入参数, 查询的结束位置。
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方无停止线\n
     * - 其他 处理失败
     */
    Av3hrCode GetStopLines(std::vector<caic_map::Profile*>& stop_lines, const uint32_t& path_id,
                           const uint32_t& start_offset, const uint32_t& end_offset);

    /**
     * @brief 获取车辆前方规划路径上的红绿灯信息
     * @param[out] traffic_lights 输出参数，前方红绿灯的信息的profile
     * @param[in] path_id 输入参数，主路经id
     * @param[in] start_offset 输入参数，查询的起始位置,
     * @param[in] end_offset 输入参数，查询的结束位置
     * @return Av3hrCode
     * -   0 成功\n
     * -   7 发送范围内前方无红绿灯\n
     * - 其他 处理失败
     */
    Av3hrCode GetTrafficLight(std::vector<caic_map::Profile*>& traffic_lights, const uint32_t& path_id,
                              const uint32_t& start_offset, const uint32_t& end_offset);

    /**
     * @brief 根据对应的link_id查找对应的link_info(耗时较长，全量查找)
     * @param[in]  link_id
     * @param[out] link_info link的信息
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode GetLinkInfo(const uint64_t link_id, RoadModel& link_info);
    /**
     * @brief 根据对应的link_id查找对应的lane_group(耗时较长，全量查找)
     * @param[in] link_id link_id
     * @param[out]  lane_group
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode GetLaneGroup(const uint64_t link_id, LaneGroup& lane_group);
    /**
     * @brief 根据对应的lane_id查找对应的lane_info
     * @param[out] lane_info lane的信息
     * @param[in]  lane_id
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode GetLaneInfo(const uint64_t lane_id, LaneInfoX& lane_info);
    // 获取驶出道路
    Av3hrCode GetOutLinks(const uint64_t link_id, std::vector<RoadModel>& link_infos);
    // 获取驶入道路
    Av3hrCode GetInLinks(const uint64_t link_id, std::vector<RoadModel>& link_infos);

    /**
     * @brief 获取当前车道的后继车道，基于maplib保存的最新更新的位置
     * @param[out] lanes_info  所有的laneinfo
     * @return Av3hrCode
     * -   0 成功\n
     * -   10 找到匹配的laneid，但是没有对应的laneinfoX数据 \n
     * - 其他 处理失败
     */
    Av3hrCode GetSuccessorLanes(std::vector<LaneInfoX>& lanes_info);

    /**
     * @brief 基于maplib保存的最新位置所在车道的前驱车道
     * @param[out] lanes_info  所有的laneinfo
     * @return Av3hrCode
     * -   0 成功\n
     * -   10 找到匹配的laneid，但是没有对应的laneinfoX数据 \n
     * - 其他 处理失败
     */
    Av3hrCode GetPredecessorLanes(std::vector<LaneInfoX>& lanes_info);
    // 获取后继车道
    /**
     * @brief 获取当前车道的后继车道，基于maplib保存的最新更新的位置
     * @param[out] lanes_info  所有的laneinfo
     * @return Av3hrCode
     * -   0 成功\n
     * -   10 找到匹配的laneid，但是没有对应的laneinfoX数据 \n
     * - 其他 处理失败
     */
    Av3hrCode GetSuccessorLanes(uint64_t lane_id, std::vector<LaneInfoX>& lanes_info);
    // 获取前驱车道
    /**
     * @brief 获取当前车道的后继车道，基于maplib保存的最新更新的位置
     * @param[out] lanes_info  所有的laneinfo
     * @return Av3hrCode
     * -   0 成功\n
     * -   10 找到匹配的laneid，但是没有对应的laneinfoX数据 \n
     * - 其他 处理失败
     */
    Av3hrCode GetPredecessorLanes(uint64_t lane_id, std::vector<LaneInfoX>& lanes_info);

    /**
     * @brief 获取当前车道的后继links，基于maplib保存的最新更新的位置
     * @param[out] links_info  所有后继的link
     * @return Av3hrCode
     * -   0 成功\n
     * -   10 找到匹配的link_id，但是没有对应的RoadModel数据
     * - 其他 处理失败
     */
    Av3hrCode GetSuccessorLinks(std::vector<RoadModel>& links_info);
    /**
     * @brief 获取当前车道的后继links，基于输入的link_id
     * @param[in] link_id
     * @param[out] links_info  所有后继的link
     * @return Av3hrCode
     * -   0 成功\n
     * -   10 找到匹配的link_id，但是没有对应的找到匹配的link_id数据
     * - 其他 处理失败
     */
    Av3hrCode GetSuccessorLinks(uint64_t link_id, std::vector<RoadModel>& links_info);
    /**
     * @brief 获取当前车道的前驱links，基于maplib维护的最新的坐标
     * @param[out] links_info  所有前驱的link
     * @return Av3hrCode
     * -   0 成功\n
     * -   10 找到匹配的link_id，但是没有对应的找到匹配的link_id数据
     * - 其他 处理失败
     */
    Av3hrCode GetPredecessorLinks(std::vector<RoadModel>& links_info);
    /**
     * @brief 获取当前车道的前驱links，基于输入的link_id
     * @param[in] link_id
     * @param[out] links_info  所有前驱的link
     * @return Av3hrCode
     * -   0 成功\n
     * -   10 找到匹配的link_id，但是没有对应的找到匹配的link_id数据
     * - 其他 处理失败
     */
    Av3hrCode GetPredecessorLinks(uint64_t link_id, std::vector<RoadModel>& links_info);

    // todo:快速vp定位接口
    Av3hrCode RapidMapPositioning(const WGS84Point& pt, const double heading, PositionX& pos);

    /**
     * @brief 获取坐标所在的link的lanegroup
     * @param[in]Pos 坐标
     * @param[out] laneGroup_info lanes的合集
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode GetNearestLane(const PositionX& Pos, LaneGroup& laneGroup_info);
    /**
     * @brief 获取坐标所在的link的前方range范围内lanegroup,(PS:并不会返回当前车道所在的laneGroup)
     * @param[in]Pos 坐标
     * @param[in]range 范围，单位为cm
     * @param[out] laneGroups_info laneGroup的合集
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode GetNearestLane(const PositionX& Pos, uint32_t range, std::vector<LaneGroup>& laneGroups_info);

    /**
     * @brief 获取车道长度（定位使用）
     * @param [in] lane 车道
     * @param [out] length 长度：单位（cm）
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode GetLaneLength(const LaneInfoX& lane, uint32_t& length);   // <- unit:

    /**
     * @brief 获取车道宽度（定位使用）
     * @param [in] Pos 位置信息
     * @param [out] length 宽度：单位（cm）
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    // todo: 如果大融合不使用，考虑移除该功能，将该功能放置于车道匹配模块
    Av3hrCode GetLaneWidth(const PositionX& Pos, uint32_t& width);

    /**
     * @brief 获取坐标相对车道中心线的偏移（定位使用）
     * @param [in] pos 位置信息
     * @param [in] lane 车道信息
     * @param [out] offset 偏移量：单位（cm），相对偏移量
     * @param [out] is_left 是否在中心线左边
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    // todo: 如果大融合不使用，考虑移除该功能，将该功能放置于车道匹配模块
    Av3hrCode GetHorizonOffset(const PositionX& pos, const LaneInfoX& lane, uint32_t& offset, bool& is_left);

    // this API function is contained by API below(GetLaneBoundaryType)
    // Av3hrCode GetLaneBoundaryIsVirtual(const LaneInfoX& lane, bool& left_virtual, bool& right_virtual);

    /**
     * @brief 获取车道边界线的点，所有点在offset之后
     * @param [in] lane 车道数据
     * @param [in] offset 偏移相对于这一段的lane/link起点的offset
     * @param [in] is_left true返回左边，false返回右边
     * @param [out] points 车道边界线所有点的坐标,UTM坐标系，标准UTM坐标
     * @return Av3hrCode
     * -   0 成功
     * - 其他 处理失败
     */
    Av3hrCode GetLaneBoundaryPoints(const LaneInfoX& lane, const uint32_t& offset, std::vector<UTMPoint>& points,
                                    bool is_left);

    /**
     * @brief 获取车道边界线的类型
     * @param [in] lane 车道数据
     * @param [in] is_left true 返回左侧车道边界线，false返回右侧车道边界线
     * @param [out] type 枚举类
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode GetLaneBoundaryType(const LaneInfoX& lane, bool is_left, BoundaryType& type);

    //    /**
    //     * @brief 获取车道边界线是否是curb
    //     * @param [in] lane 车道信息
    //     * @param [in] is_left true 返回左侧车道边界线，false返回右侧车道边界线
    //     * @param [out] is_curb true为curb
    //     * @return Av3hrCode
    //     * -   0 成功\n
    //     * - 其他 处理失败
    //     */
    //    Av3hrCode IsLaneBoundaryCurb(const LaneInfoX& lane, bool is_left, bool is_curb);
    /**
     * @brief 获取WGS84坐标对应所在的link下的offset，供给融合定位使用
     * @param [in] UTME，utm坐标（标准的UTM坐标，无偏移）
     * @param [in] UTMN，utm坐标（标准的UTM坐标，无偏移）
     * @param [in] config config "debug"时启用log，使用cout输出
     * @param [out] pos 包含offset等属性的坐标类
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode UTM2Offset(double UTME, double UTMN, PositionX& Pos, std::string config = "");

    /**
     * @brief 获取WGS84坐标对应所在的link下的offset,快速版本，默认在与maplib保存的最近定位点path_id一致
     * @param [in] UTME，utm坐标（标准的UTM坐标，无偏移）
     * @param [in] UTMN，utm坐标（标准的UTM坐标，无偏移）
     * @param [in] config config "debug"时启用log，使用cout输出
     * @param [out] pos 包含offset等属性的坐标类
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode UTM2OffsetRapid(double UTME, double UTMN, PositionX& Pos, std::string config = "");




    /**
     * @brief 获取车道对应的对向车道
     * @param [in] lane_id，lane_id
     * @param [out] lane_boundary，lane_id车道对应的对向车道
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode GetReverseLanes(const uint16_t& lane_id, std::vector<UTMPoint>& lane_boundary);



private:
    // TODO(cuidengji) : 后续统一用 interface/caic_std.h中的类型，故命名先保持一致，便于后续切换
    struct Point2d
    {
        double x;
        double y;

        Point2d(const double x_in, const double y_in)
        {
            x = x_in;
            y = y_in;
        }
        Point2d()
        {
            x = 0.0;
            y = 0.0;
        }
    };

    /**
     * @brief 找到点到直线的最近距离,不考虑高程
     * @param [in] point 待判定的点
     * @param [in] line_point1,line_point2 直线点
     * @param [out] point_projected 点在直线的投影点
     * @param [out] point_projected 点在直线的投影点
     * @param [out] haspointInner 点是否在两点连成的线段上
     * @return double 距离
     */
    double FindDistance2D(Point2d point, const Point2d& line_point1, const Point2d& line_point2,
                          Point2d& point_projected, bool& bOnSegment);

    bool OnSegment(const Point2d& point, const Point2d& line_point1, const Point2d& line_point2);

    /**
     * @brief 判断点在向量p1p2的左边还是右边
     * @param [in] point 待判定的点
     * @param [in] line_point1,line_point2 向量p1p2
     * @return int -1在左边，0在向量的方向上，1在右边
     */
    int GetPointOrientation(const Point2d& point, const Point2d& v_point1, const Point2d& v_point2);

    // TODO: 暂时放在私有部分,按需再考虑是否public
    /**
     * @brief  从车道边界线属性中转换BoundaryType
     * @param [in] LaneBoundaryAttribute 车道边界线属性数据
     * @param [out] type 枚举类
     * @return Av3hrCode
     * -   0 成功\n
     * - 其他 处理失败
     */
    Av3hrCode FormatLineMarkingToBoundaryType(const LaneBoundaryAttribute& laneBoundaryAttribute, BoundaryType& type);

    Av3hrCode GetLinePointsByOffset(const std::vector<WGS84Point>& InPoints, const std::uint32_t Offset,
                                    std::vector<UTMPoint>& OutPoints);

    bool PointToLineDistance(std::vector<WGS84Point>& linePoint, Point2d& locPoint, double& dis_min, Point2d& proj,
                             int& num, int& cent_ori, int &iMethod,bool bPrint = false);

};   // class Av3hrInterface
}   // namespace maplib
}   // namespace caic_map
