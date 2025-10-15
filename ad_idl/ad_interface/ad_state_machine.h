#ifndef ad_INTERFACE_ad_STATE_MACHINE_H
#define ad_INTERFACE_ad_STATE_MACHINE_H

#include <stdint.h>
#include <string>
#include <vector>

namespace ad_state_machine {
/**
 * @brief now state of state machine 
 * 
 */
enum struct NowState : uint8_t {
  UNKNOWN = 0,
  MD_SYSTEM_PASSIVE,
  MD_PILOT_STANDBY,
  ENGAGED_NOA_HIGHWAY,
  ENGAGED_NOA_URBAN,
  MD_MPA_STANDBY,
  ENGAGED_MPA_ACTIVE,
  MD_HPA_STANDBY,
  ENGAGED_HPA_ACTIVE,
  MD_APA_STANDBY,
  ENGAGED_APA_ACTIVE,
  SLAM_STANDBY,
  SLAM_ACTIVE,
};

/**
 * @brief Set_CDASteerAssist
 * CDA转向辅助
 */
enum struct SetCdaSteerAssist : uint8_t {
  ACC = 0,  /**< ACC*/
  IACC,     /**< iACC*/
  NOA,      /**< NOA+*/
};

/**
 * @brief FunctionRequest
 * 功能请求
 */
enum struct FunctionRequest : uint8_t {
  PASSIVE = 0,  /**< FunctionRequest System Passive*/
  LOADED,       /**< FunctionRequest System Loaded*/
  CHECKING,     /**< FunctionRequest System checking*/
  STANDBY,      /**< FunctionRequest System standby*/
  RUNNING,      /**< FunctionRequest System running*/
};

struct FunctionRequestSet {
  FunctionRequest cda_function_request;     /**< CDA功能请求*/
  FunctionRequest noa_function_request;     /**< NOA功能请求*/
  FunctionRequest urban_function_request;   /**< Urban功能请求*/
  FunctionRequest mpa_function_request;     /**< MPA功能请求*/
  FunctionRequest apa_function_request;     /**< APA功能请求*/
  FunctionRequest hpa_function_request;     /**< HPA功能请求*/
  FunctionRequest slam_function_request;    /**< SLAM功能请求*/
};

/**
 * @brief CDA_Inhibit
 * CDA阻碍
 */ 
enum struct CdaInhibit : uint8_t {
  NO_ACTIVE_INHIBIT = 0,            /**< no active Inhibit*/
  ACTIVE_SOFT_INHIBIT_AD,           /**< active Soft Inhibit: AD(Attention Demand)*/
  ACTIVE_SOFT_INHIBIT_TD,           /**< active Soft Inhibit: TD(Takeover Demand)*/
  ACTIVE_HARD_INHIBIT_EAS,          /**< active Hard Inhibit: EAS(Emergency Automatic Stop)*/
  ACTIVE_HARD_INHIBIT_INSTANTOFF,   /**< active Hard Inhibit: Instant-Off*/
};

/**
 * @brief ActivationPrevention
 * 防止激活
 */ 
enum struct ActivationPrevention : uint8_t {
  ACTIVATION = 0,       /**< all activation conditions ok*/
  NOT_ACTIVATION,       /**< not all activation conditions ok*/
};

struct ActivationPreventionSet {
  ActivationPrevention cda_activation_prevention;       /**< CDA防止激活*/
  ActivationPrevention noa_activation_prevention;       /**< NOA防止激活*/
  ActivationPrevention urban_activation_prevention;     /**< Urban防止激活*/
  ActivationPrevention mpa_activation_prevention;       /**< MPA防止激活*/
  ActivationPrevention apa_activation_prevention;       /**< APA防止激活*/
  ActivationPrevention hpa_activation_prevention;       /**< HPA防止激活*/
  ActivationPrevention slam_activation_prevention;      /**< SLAM防止激活*/
};

/**
 * @brief NOA_Status
 * NOA状态
 */ 
enum struct NoaStatus : uint8_t {
  UNAVAILABLE = 0,              /**< NOA Status unavailable*/
  AVAILABLE,                    /**< NOA Status available*/
  REQUEST_TO_ENGAGE_URBAN,      /**< NOA Status requesting to engage NOA Urban*/
  REQUEST_TO_ENGAGE_HPA,        /**< NOA Status requesting to engage HPA*/
  REQUEST_TO_ENGAGE_MPA,        /**< NOA Status requesting to engage MPA*/
};

/**
 * @brief Urban_Status
 * Urban状态
 */ 
enum struct UrbanStatus : uint8_t {
  UNAVAILABLE = 0,              /**< NOA Urban Status unavailable*/
  AVAILABLE,                    /**< NOA Urban Status available*/
  REQUEST_TO_ENGAGE_NOA,        /**< NOA Urban Status requesting to engage NOA*/
  REQUEST_TO_ENGAGE_HPA,        /**< NOA Urban Status requesting to engage HPA*/
  REQUEST_TO_ENGAGE_MPA,        /**< NOA Urban Status requesting to engage MPA*/
};

/**
 * @brief MPA_Status
 * MPA状态
 */ 
enum struct MpaStatus : uint8_t {
  UNAVAILABLE = 0,               /**< MPA Status unavailable*/
  AVAILABLE,                     /**< MPA Status available*/
  REQUEST_TO_ENGAGE_URBAN,       /**< MPA Status requesting to engage NOA Urban*/
  REQUEST_TO_ENGAGE_HPA,         /**< MPA Status requesting to engage HPA*/
  REQUEST_TO_ENGAGE_NOA,         /**< MPA Status requesting to engage NOA*/
};

/**
 * @brief APA_Status
 * APA状态
 */ 
enum struct ApaStatus : uint8_t {
  UNAVAILABLE = 0,              /**< APA Status unavailable*/
  AVAILABLE,                    /**< APA Status available*/
};

/**
 * @brief HPA_Status
 * HPA状态
 */ 
enum struct HpaStatus : uint8_t {
  UNAVAILABLE = 0,              /**< HPA Status unavailable*/
  AVAILABLE,                    /**< HPA Status available*/
  REQUEST_TO_ENGAGE_NOA,        /**< HPA Status requesting to engage NOA*/
  REQUEST_TO_ENGAGE_URBAN,      /**< HPA Status requesting to engage NOA Urban*/
  REQUEST_TO_ENGAGE_MPA,        /**< HPA Status requesting to engage MPA*/
};

/**
 * @brief Suppression
 * 抑制
 */ 
enum struct Suppression : uint8_t {
  NO_SUPPRESSION = 0,       /**< no Suppression*/    
  ACT_SUPPRESSION,          /**< act Suppression*/
};

struct SuppressionSet {
  Suppression noa_suppression;        /**< NOA抑制*/
  Suppression urban_suppression;      /**< Urban抑制*/
  Suppression mpa_suppression;        /**< MPA抑制*/
  Suppression apa_suppression;        /**< APA抑制*/
  Suppression hpa_suppression;        /**< HPA抑制*/
  Suppression slam_suppression;       /**< SLAM抑制*/
};

/**
 * @brief SetCdaApa
 * 
 */ 
enum struct SetCdaApa : uint8_t {
  APA_OFF = 0,        
  APA_ON,          
};

struct StateMachine : public ad_std::MessageBase {
  ad_std::HeaderPOD header; 
  SetCdaSteerAssist set_cda_steer_assist;
  FunctionRequestSet function_request_set;  
  CdaInhibit cda_inhibit;
  ActivationPreventionSet activation_prevention_set;
  NoaStatus noa_status;
  UrbanStatus urban_status;
  MpaStatus mpa_status;
  ApaStatus apa_status;
  HpaStatus hpa_status;
  SuppressionSet suppression_set;
};

}  // namespace ad_state_machine
#endif
