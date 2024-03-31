//
// Created by xucong on 24-1-3.
//

#ifndef SIM_INTERFACE_SIM_GROUND_TRUTH_H_
#define SIM_INTERFACE_SIM_GROUND_TRUTH_H_

#include "sim_std.h"

namespace sim_ground_truth {

#define OBJECT_INFO_SIZE 4
#define TLIGHT_INFO_SIZE 4
#define LANE_INFO_SIZE   4

/** @addtogroup GENERAL_DEFINITIONS
 *  @{
 */
#define DEFAULT_PORT   48190       /**< default port for RDB communication     */
#define FEEDBACK_PORT  48191       /**< port for RDB feedback to taskControl   */
#define IMAGE_PORT     48192       /**< port for RDB image data                */
#define MAGIC_NO       35712       /**< magic number                           */
#define VERSION       0x0125       /**< upper byte = major, lower byte = minor */
/** @} */

#define RDB_SIZE_OBJECT_NAME       32
/** @addtogroup RDB_OBJECT_CATEGORY
 *  ------ object types ------
 *  @{
 */
#define OBJECT_CATEGORY_NONE           0    /**< no category defined          */
#define OBJECT_CATEGORY_PLAYER         1    /**< category is player           */
#define OBJECT_CATEGORY_SENSOR         2    /**< category is sensor           */
#define OBJECT_CATEGORY_CAMERA         3    /**< category is camera           */
#define OBJECT_CATEGORY_LIGHT_POINT    4    /**< category is light point      */
#define OBJECT_CATEGORY_COMMON         5    /**< category is common object    */
#define OBJECT_CATEGORY_OPENDRIVE      6    /**< category is OpenDRIVE object */
/** @} */

/** @addtogroup RDB_OBJECT_TYPE
 *  ------ object types ------
 *  @{
 */
#define OBJECT_TYPE_NONE                   0    /**< undefined object type for categories other than player  */
#define OBJECT_TYPE_PLAYER_NONE            0    /**< undefined player type             			 		             */
#define OBJECT_TYPE_PLAYER_CAR             1    /**< player of type car                                      */
#define OBJECT_TYPE_PLAYER_TRUCK           2    /**< player of type truck                                    */
#define OBJECT_TYPE_PLAYER_VAN             3    /**< player of type van                                      */
#define OBJECT_TYPE_PLAYER_BIKE            4    /**< player of type bicycle                                  */
#define OBJECT_TYPE_PLAYER_PEDESTRIAN      5    /**< player of type pedestrian                               */
#define OBJECT_TYPE_PLAYER_PED_GROUP       6    /**< player of type pedestrian group                         */
#define OBJECT_TYPE_POLE                   7    /**< pole                                                    */
#define OBJECT_TYPE_TREE                   8    /**< tree                                                    */
#define OBJECT_TYPE_BARRIER                9    /**< barrier                                                 */
#define OBJECT_TYPE_OPT1                  10    /**< optional user type 1                                    */
#define OBJECT_TYPE_OPT2                  11    /**< optional user type 2                                    */
#define OBJECT_TYPE_OPT3                  12    /**< optional user type 3                                    */
#define OBJECT_TYPE_PLAYER_MOTORBIKE      13    /**< player of type motorbike                                */
#define OBJECT_TYPE_PLAYER_BUS            14    /**< player of type bus                                      */
#define OBJECT_TYPE_STREET_LAMP           15    /**< street lamp                                             */
#define OBJECT_TYPE_TRAFFIC_SIGN          16    /**< traffic sign                                            */
#define OBJECT_TYPE_HEADLIGHT             17    /**< headlights                                              */
#define OBJECT_TYPE_PLAYER_TRAILER        18    /**< player of type trailer                                  */
#define OBJECT_TYPE_BUILDING              19    /**< object of type building                                 */
#define OBJECT_TYPE_PARKING_SPACE         20    /**< object of type parking space                            */
#define OBJECT_TYPE_ROAD_WORKS            21    /**< object for road works                                   */
#define OBJECT_TYPE_ROAD_MISC             22    /**< miscellaneous road object                               */
#define OBJECT_TYPE_TUNNEL                23    /**< object for tunnel environment                           */
#define OBJECT_TYPE_LEGACY                24    /**< legacy object (not to be used)                          */
#define OBJECT_TYPE_VEGETATION            25    /**< common vegetation object                                */
#define OBJECT_TYPE_MISC_MOTORWAY         26    /**< common motorway object                                  */
#define OBJECT_TYPE_MISC_TOWN             27    /**< common town object                                      */
#define OBJECT_TYPE_PATCH                 28    /**< patch on road                                           */
#define OBJECT_TYPE_OTHER                 29    /**< any other unspecified object                            */
#define OBJECT_PLAYER_SEMI_TRAILER        30    /**< player of type semi trailer                             */
#define OBJECT_PLAYER_RAILCAR             31    /**< player of type rail car                                 */
#define OBJECT_PLAYER_RAILCAR_SEMI_HEAD   32    /**< player of type rail car semi, head                      */
#define OBJECT_PLAYER_RAILCAR_SEMI_BACK   33    /**< player of type rail car semi, back                      */
#define OBJECT_TYPE_VEH_LIGHT_FRONT_LEFT  34    /**< headlight front left                                    */
#define OBJECT_TYPE_VEH_LIGHT_FRONT_RIGHT 35    /**< headlight front right                                   */
#define OBJECT_TYPE_VEH_LIGHT_REAR_LEFT   36    /**< tail light left                                         */
#define OBJECT_TYPE_VEH_LIGHT_REAR_RIGHT  37    /**< tail light right                                        */
#define OBJECT_TYPE_VEH_CABIN             38    /**< articulated cabin (e.g. for trucks), must have parent   */
/** @} */

/** @addtogroup RDB_ROADMARK_TYPE
 *  ------ road mark types ------
 *  @{
 */
#define ROADMARK_TYPE_NONE           0      /**< no roadmark defined */
#define ROADMARK_TYPE_SOLID          1      /**< solid marks         */
#define ROADMARK_TYPE_BROKEN         2      /**< broken marks        */
#define ROADMARK_TYPE_CURB           3      /**< curb                */
#define ROADMARK_TYPE_GRASS          4      /**< grass               */
#define ROADMARK_TYPE_BOTDOT         5      /**< Botts' dots         */
#define ROADMARK_TYPE_OTHER          6      /**< something else      */
#define ROADMARK_TYPE_SOLID_SOLID    7      /**< solid solid         */
#define ROADMARK_TYPE_BROKEN_SOLID   8      /**< broken solid        */
#define ROADMARK_TYPE_SOLID_BROKEN   9      /**< solid broken        */
#define ROADMARK_TYPE_LANE_CENTER   10      /**< center of the lane  */
/** @} */

/** @addtogroup RDB_ROADMARK_COLOR
 *  ------ road mark colors ------
 *  @{
 */
#define ROADMARK_COLOR_NONE          0      /**< no color defined */
#define ROADMARK_COLOR_WHITE         1      /**< white color      */
#define ROADMARK_COLOR_RED           2      /**< red color        */
#define ROADMARK_COLOR_YELLOW        3      /**< yellow color     */
#define ROADMARK_COLOR_OTHER         4      /**< other color      */
#define ROADMARK_COLOR_BLUE          5      /**< blue color       */
#define ROADMARK_COLOR_GREEN         6      /**< green color      */
/** @} */



/** ------ header of a complete message ------ */
struct MsgHeader {
  uint16_t  magicNo;      /**< must be RDB_MAGIC_NO (35712)                                               @unit       */
  uint16_t  version;      /**< upper byte = major, lower byte = minor                                     @unit _     */
  uint32_t  headerSize;   /**< size of this header structure when transmitted                             @unit byte  */
  uint32_t  dataSize;     /**< size of data following the header                                          @unit byte  */
  uint32_t  frameNo;      /**< number of the stoic frame                                             @unit _     */
  double    simTime;      /**< stoic time                                                            @unit s     */
};

/** ------ geometry information for an object --- */
struct Geometry
{
  float dimX;        /**< x dimension in object co-ordinates (length)     @unit m  */
  float dimY;        /**< y dimension in object co-ordinates (width)      @unit m  */
  float dimZ;        /**< z dimension in object co-ordinates (height)     @unit m  */
};

/**
 * @brief the lane cubic polynomial
 * y=c0+c1*x+c2*x*x+c3*x*x*x
 */
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

/** ------ complete object data (basic and extended info) ------- */
struct ObjectData
{
  uint32_t            id;                         /**< unique object ID witin the given category                     @unit _   */
  uint8_t             category;                   /**< object category                                               @unit     */
  uint8_t             type;                       /**< object type                                                   @unit     */
  float               heading;                    /**< heading angle                                                 @unit rad */
  Geometry            geo;                        /**< info about object's geometry                                  @unit m   */
  sim_std::Point3d    pos;                        /**< position and orientation of object's reference point          @unit     */
  sim_std::Vector3d   speed;                      /**< speed and rates                                               @unit     */
  sim_std::Vector3d   accel;                      /**< acceleration                                                  @unit     */
};

/** ------ information about a traffic light (state) ------ */
struct TrafficLight
{
  int32_t                   id;                             /**< unique ID of the traffic light                                      */
  float                     state;                          /**< current state (normalized)                                          */
  uint32_t                  stateMask;                      /**< current state mask (light mask, e.g. for gfx)                       */
  int32_t                   ctrlId;                         /**< ID of the traffic light's controller                                */
  float                     cycleTime;                      /**< duration of a complete cycle of all phases                          */
  uint16_t                  noPhases;                       /**< number of phases provided by this traffic light                     */
  float                     duration;                       /**< normalized duration of the phase, invalid phases will have duration */
  uint8_t                   type;                           /**< type of the phase                                                   */
};

/** ------ lane line information ------
 * @note this package is immediately followed by "noDataPoints" entries of type RDB_POINT_t
 */
struct LaneLine
{
  uint32_t            playerId;                /**< id of the player to which roadmark belongs                       @unit _  */
  int8_t              id;                      /**< id of this road mark                                             @unit    */
  int8_t              laneId;                  /**< id of the lane to which the roadmark belongs                     @unit _  */
  uint8_t             type;                    /**< type of road mark                                                @unit    */
  uint8_t             color;                   /**< color of road mark                                               @unit    */
  uint16_t            noDataPoints;            /**< number of tesselation points following this package              @unit _  */
  uint32_t            roadId;                  /**< id of the road to which the roadmark belongs                     @unit _  */
  int8_t           curLaneId;               /**< for current lane id                                              @unit _  */
  LaneCubicPolynomial poly;
};

/** ------ front vision object data ------
 */
struct FrontVisionObjects {
  MsgHeader header;                           /**< header of message >**/
  ObjectData object[OBJECT_INFO_SIZE];        /**< object data >**/
  uint8_t objects_size;                       /**<actual used size */
  TrafficLight light[TLIGHT_INFO_SIZE];       /**< traffic light data >**/
  uint8_t lights_size;                       /**<actual used size */
};

/** ------ bev front wide vision object data ------
 */
struct FrontWideVisionObjects {
  MsgHeader header;                           /**< header of message >**/
  ObjectData object[OBJECT_INFO_SIZE];        /**< object data >**/
  uint8_t objects_size;                       /**<actual used size */
  TrafficLight light[TLIGHT_INFO_SIZE];       /**< traffic light data >**/
  uint8_t lights_size;                       /**<actual used size */
};

/** ------ bev front right vision object data ------
 */
struct FrontRightVisionObjects {
  MsgHeader header;                           /**< header of message >**/
  ObjectData object[OBJECT_INFO_SIZE];        /**< object data >**/
  int objects_size;
  TrafficLight light[TLIGHT_INFO_SIZE];       /**< traffic light data >**/
  uint8_t lights_size;                       /**<actual used size */
};

/** ------ bev front left vision object data ------
 */
struct FrontLeftVisionObjects {
  MsgHeader header;                           /**< header of message >**/
  ObjectData object[OBJECT_INFO_SIZE];        /**< object data >**/
  int objects_size;
  TrafficLight light[TLIGHT_INFO_SIZE];       /**< traffic light data >**/
  uint8_t lights_size;                       /**<actual used size */
};

/** ------ front left vision object data ------
 */
struct RearVisionObjects {
  MsgHeader header;                           /**< header of message >**/
  ObjectData object[OBJECT_INFO_SIZE];        /**< object data >**/
  int objects_size;
};

/** ------ front left vision object data ------
 */
struct RearRightVisionObjects {
  MsgHeader header;                           /**< header of message >**/
  ObjectData object[OBJECT_INFO_SIZE];        /**< object data >**/
  int objects_size;
};

/** ------ front left vision object data ------
 */
struct RearLeftVisionObjects {
  MsgHeader header;                           /**< header of message >**/
  ObjectData object[OBJECT_INFO_SIZE];        /**< object data >**/
  int objects_size;
};

/** ------ traffic light data ------
 */
struct TrafficLights {
  MsgHeader header;                              /**< header of message >**/
  TrafficLight traffic_light[TLIGHT_INFO_SIZE];  /**< traffic light data >**/
  int tlights_size;
};

/** ------ lane line data ------
 */
struct LaneLines {
  MsgHeader header;                           /**< header of message >**/
  LaneLine lane[LANE_INFO_SIZE];              /**< lane line data >**/
  int lane_size;
};


}

#endif //SIM_INTERFACE_SIM_GROUND_TRUTH_H_
