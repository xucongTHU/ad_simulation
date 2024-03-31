#ifndef ad_INTERFACE_ad_HMI_INTERFACE_H
#define ad_INTERFACE_ad_HMI_INTERFACE_H

#include <stdint.h>

#include "ad_interface/ad_interaction.h"

namespace ad_hmi {

    enum struct HmiSwitchStatus : uint8_t {
        OFF = 0, // 关
        ON, // 开
    };


    enum struct HmiParkStatusFunctionRequest : uint8_t {
        PASSIVE = 0,  // 功能抑制
        LOADED,
        CHECKING, // 检测中
        STANDBY, // 功能就绪
        RUNNING, // 运行中
    };
    enum struct HmiSlamRouteType : uint8_t {
        NO_TYPE = 0, // 无类型
        ENTER, // 入库
        OUT,  // 出库
        ENTER_OUT,  // 出入库
    };
    enum struct HmiHpaSlamPointSet : uint8_t {
        NO_S_POINT = 0, // 未设置起点
        SET_S_POINT = 1, // 设置起点
        NO_E_POINT = 3, // 未设置终点
        SET_E_POINT = 4, // 设置终点
    };
    struct HmiConfigStatusSet : public ad_std::MessageBase {
        HmiSwitchStatus cda_lcc_switch;     // lcc 开关
        HmiSwitchStatus cda_acc_switch;  // acc 开关
        HmiSwitchStatus cda_noa_switch;  // noa 开关
        HmiSwitchStatus cda_mpa_switch;  // 记忆行车配置开关
        HmiSwitchStatus cda_dtlc_switch;    //  dtlc 开关
        HmiSwitchStatus ras_select_status;    //  ras  HmiSwitchStatus::OFF Not select HmiSwitchStatus::ON select
        HmiSwitchStatus cda_hpa_button;  //  hpa按钮,行泊切换 HmiSwitchStatus::OFF 关 HmiSwitchStatus::ON开
        HmiSwitchStatus mpa_function_button;  // 记忆行车功能按钮 HmiSwitchStatus::OFF 关 HmiSwitchStatus::ON开
        HmiSwitchStatus hpa_switch; //  hpa 开关
        HmiSwitchStatus rpa_switch; // rpa 开关
        HmiSwitchStatus ras_continue_status; // RAS暂停后是否继续 HmiSwitchStatus::OFF 否 HmiSwitchStatus::ON 是
        HmiSwitchStatus mpa_mr_create_route; //创建路线图标点击状态
        HmiSwitchStatus mpa_mr_complete_status; //路线记忆"完成"图标点击状态
        HmiSwitchStatus mpa_mr_cancle_status; //路线记忆"取消"图标点击状态
        HmiSwitchStatus mpa_mr_mark_status; //"确认收藏"点击状态
        ad_interaction::Apa apa_select_status;  //    Apa.is_active 是否激活
        ad_interaction::Apa apa_park_out_dir;  //     Apa.in_wards Apa.out_left 泊出方向
        HmiParkStatusFunctionRequest apa_park_search_request; // APA车位搜索功能请求
        HmiParkStatusFunctionRequest apa_park_in_request; // APA泊入功能请求
        HmiParkStatusFunctionRequest apa_park_out_request; // APA泊出功能请求
        HmiParkStatusFunctionRequest hpa_function_request; // HPA功能请求
        HmiHpaSlamPointSet hpa_slam_point_set; // HPA建图起终点设置状态
        HmiSwitchStatus hpa_slam_storage_status; // HPA建图存储空间/条数限制状态 0 不可用 1 可用
        HmiSwitchStatus hpa_slam_mark_status; // HPA建图路线确认收藏状态 0 不收藏 1收藏
        HmiSlamRouteType hpa_route_type; // HPA建图创建路线类型
        ad_interaction::Avp hpa_route_select; // HPA用户选择路线信息   Avp.parking_space_id 路线  Avp.is_active 路线选择状态
        HmiSwitchStatus mpa_start_driving; //“出发”图标点击状态
        HmiSwitchStatus mpa_mr_use_route; //“使用”路线图标点击状态
        HmiSwitchStatus mpa_mr_ignore_route;//“忽略”路线图标点击状态
        ad_interaction::Status mpa_function_request; //MPA功能请求
    };


} // namespace ad_hmi

#endif
