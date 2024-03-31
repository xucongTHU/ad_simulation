//
// Created by xucong on 23-12-28.
//
#include "simulator/vtd_integration/front_vision/app/front_vision_model.h"

namespace stoic::simulator {
int FrontVisionModel::openPort(int &descriptor, int portNo, const char *serverAddr) {
  struct sockaddr_in server{};
  struct hostent *host = nullptr;
  //
  // Create the socket, and attempt to connect to the server
  //
  descriptor = socket(AF_INET, SOCK_DGRAM, 0);
  if (descriptor == -1) {
    LOG(ERROR) << "openPort: socket() failed: " << strerror(errno);
    return 0;
  }
  int opt = 1;
  if (setsockopt(descriptor, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt))) {
    LOG(ERROR) << "setsockopt() failed: " << strerror( errno );
  }
  server.sin_family      = AF_INET;
  server.sin_port        = htons(portNo);
  server.sin_addr.s_addr = inet_addr(serverAddr);
  if (server.sin_addr.s_addr == INADDR_NONE) {
    host = gethostbyname(serverAddr);
    if (host == nullptr) {
      LOG(ERROR) << "openPort: unable to resolve server: " << serverAddr;
      return 0;
    }
    memcpy(&server.sin_addr, host->h_addr_list[0], host->h_length);
  }
  // wait for connection
  if (bind(descriptor, (struct sockaddr *)&server, sizeof(server)) < 0) {
    LOG(ERROR) << "bind failed: " << strerror( errno );
  }
  LOG(ERROR) << "port " << portNo << " connected!";
  return 1;
}
void FrontVisionModel::initConnections() {
  memset( sConnection, 0, MAX_CONNECTIONS * sizeof(Connection_t));
  for ( int i = 0; i < MAX_CONNECTIONS; i++) {
    initConnection(sConnection[i]);
    sConnection[i].id = i;
  }
  sConnection[0].port = DEFAULT_PORT_TC;
  sConnection[1].port = sensor_port;
}
void FrontVisionModel::initConnection(Connection_t &conn) {
  strcpy(conn.serverAddr, sensor_ip.c_str());
  conn.desc = -1;
  conn.bufferSize = sizeof(RDB_MSG_t);
  conn.pData = (unsigned char*) calloc(1, conn.bufferSize);
}
void FrontVisionModel::readConnection(Connection_t &conn, bool waitForMessage, bool verbose) {
  // receive buffer
  static char* szBuffer = (char*) calloc(1, DEFAULT_BUFFER_SIZE);
  int ret;
  bool bMsgComplete = false;
  if ( verbose )
    LOG(INFO) << "readConnection: start reading connection " << conn.id;
  // read a complete message
  while (!bMsgComplete) {
    ret = recv(conn.desc, szBuffer, DEFAULT_BUFFER_SIZE, 0);
    if ( ret == -1 ) {
      LOG(ERROR) << "recv() failed: " << strerror( errno );
      break;
    }
    if ( verbose )
      LOG(INFO) << "readConnection: connection " << conn.id << " ret = " << ret;
    if (ret > 0) {
      if ((conn.bytesInBuffer + ret) > conn.bufferSize) {
        conn.pData = (unsigned char*) realloc(conn.pData, conn.bytesInBuffer + ret);
        conn.bufferSize = conn.bytesInBuffer + ret;
      }
      memcpy(conn.pData + conn.bytesInBuffer, szBuffer, ret);
      conn.bytesInBuffer +=ret;
      if (conn.bytesInBuffer >= sizeof(RDB_MSG_HDR_t)) {
        auto* hdr = ( RDB_MSG_HDR_t*) conn.pData;
        if (hdr->magicNo != RDB_MAGIC_NO) {
          LOG(INFO) << "message receiving is out of sync; discarding data";
          conn.bytesInBuffer = 0;
        }
        while (conn.bytesInBuffer >= (hdr->headerSize + hdr->dataSize)) {
          unsigned int msgSize = hdr->headerSize + hdr->dataSize;
          if (verbose)
            Framework::RDBHandler::printMessage((RDB_MSG_t*)conn.pData, true);
          // now parse the message
          parseRDBMessage(conn, (RDB_MSG_t*)conn.pData);
          // remove message from queue
          memmove(conn.pData, conn.pData + msgSize, conn.bytesInBuffer - msgSize);
          conn.bytesInBuffer -= msgSize;
          bMsgComplete = true;
        }
      }
    }
  }
  if (verbose)
    LOG(INFO) << "readConnection: finished reading connection " << conn.id;
}
void FrontVisionModel::parseRDBMessage(Connection_t &conn, RDB_MSG_t *msg) {
  if (!msg)
    return;
  if (!msg->hdr.dataSize)
    return;
  auto* entry = (RDB_MSG_ENTRY_HDR_t*)(((char*)msg) + msg->hdr.headerSize);
  uint32_t remainingBytes = msg->hdr.dataSize;
  while (remainingBytes) {
    parseRDBMessageEntry(conn, msg->hdr.simTime, msg->hdr.frameNo, entry);
    remainingBytes -= (entry->headerSize + entry->dataSize);
    if (remainingBytes)
      entry = (RDB_MSG_ENTRY_HDR_t*)(((char*)entry) + entry->headerSize + entry->dataSize);
  }
}
void FrontVisionModel::parseRDBMessageEntry(Connection_t &conn,
                                                const double &simTime,
                                                const unsigned int &simFrame,
                                                RDB_MSG_ENTRY_HDR_t *entryHdr) {
  if (!entryHdr)
    return;
  int noElements = entryHdr->elementSize ? (entryHdr->dataSize / entryHdr->elementSize) : 0;
  if (!noElements) {
    switch ( entryHdr->pkgId )
    {
      case RDB_PKG_ID_START_OF_FRAME:
        LOG(INFO) << "=======parseRDBMessageEntry: connection " <<  conn.id << " got start of frame=======";
        if (conn.id == 1) {  // reset information about sensor object if a new sensor frame starts.
          memset(&mObjState, 0, sizeof(RDB_OBJECT_STATE_t));
          memset(&mLaneLines, 0, sizeof(RDB_ROADMARK_t));
          memset(&mRoadPos, 0, sizeof(RDB_ROAD_POS_t));
          memset(&mLights, 0, sizeof(RDB_TRAFFIC_LIGHT_t));
          memset(&mSigns, 0, sizeof(RDB_TRAFFIC_SIGN_t));
        }
        break;
      case RDB_PKG_ID_END_OF_FRAME:
        LOG(INFO) << "=======parseRDBMessageEntry: connection " << conn.id << " got end of frame=======";
        break;
      default:
        return;
    }
    return;
  }
  int obj_size = 0;
  int lane_size = 0;
  int light_size = 0;
//  unsigned char ident = 6;
  char* dataPtr = (char*) entryHdr;
  dataPtr += entryHdr->headerSize;
  int objects = sizeof(front_vision_objects_->object) / sizeof(sim_ground_truth::ObjectData);
  int lanes = sizeof(lane_lines_->lane) / sizeof(sim_ground_truth::LaneLine);
  int lights = sizeof(traffic_lights_->traffic_light) / sizeof(sim_ground_truth::TrafficLight);
  while (noElements-- && obj_size < objects && lane_size < lanes && light_size < lights) {
    bool printedMsg = true;
    auto objPtr   = (RDB_OBJECT_STATE_t*)dataPtr;
    auto lightPtr = (RDB_TRAFFIC_LIGHT_t*)dataPtr;
    auto signPtr  = (RDB_TRAFFIC_SIGN_t *)dataPtr;
    auto linePtr  = (RDB_ROADMARK_t*)dataPtr;
    auto roadPtr  = (RDB_ROAD_POS_t*)dataPtr;
    switch (entryHdr->pkgId)
    {
      case RDB_PKG_ID_ROAD_POS:
        if (conn.id == 1 ) {
          roadHandler(simTime, simFrame, *roadPtr, conn.id == 1u);
        }
        break;
      case RDB_PKG_ID_ROADMARK:
        if (conn.id == 1 ) {
          laneHandler(simTime, simFrame, *linePtr, conn.id == 1u, lane_size);
          lane_size++;
        }
        break;
      case RDB_PKG_ID_OBJECT_STATE:
        if (conn.id == 1 && (entryHdr->flags & RDB_PKG_FLAG_EXTENDED)) {
          objHandler(simTime, simFrame, *objPtr, entryHdr->flags & RDB_PKG_FLAG_EXTENDED, conn.id == 1u, obj_size);
          obj_size++;
        }
        break;
      case RDB_PKG_ID_TRAFFIC_LIGHT:
        if (conn.id == 1 && (entryHdr->flags & RDB_PKG_FLAG_EXTENDED)) {
          tlightsHandler(simTime, simFrame, *lightPtr, entryHdr->flags & RDB_PKG_FLAG_EXTENDED, conn.id == 1u, light_size);
          light_size++;
        }
      case RDB_PKG_ID_TRAFFIC_SIGN:
        if (conn.id == 1) {
          trafficSignHandler(simTime, simFrame, *signPtr, conn.id == 1u);
        }
      default:
        printedMsg = false;
        break;
    }
    dataPtr += entryHdr->elementSize;
  }
}
void FrontVisionModel::objHandler(const double &simTime,
                                  const unsigned int &simFrame,
                                  RDB_OBJECT_STATE_t &item,
                                  bool isExtended,
                                  bool isSensor,
                                  int objSize) {
  if (item.base.id == 1 && isExtended)
    memcpy(&mOwnObject, &item, sizeof(RDB_OBJECT_STATE_t));
  else if (isExtended && isSensor)
    memcpy(&mObjState, &item, sizeof(RDB_OBJECT_STATE_t));
  LOG(INFO) << "Handle Sensor " <<  FRONT_SENSOR << " Object info";
  LOG(INFO) << "simTime = " << simTime <<  " , simFrame = " << simFrame;
  uint32_t ObjId = mObjState.base.id;
  if (objSize == 0) {
    sendObjectsUdp(simTime, simFrame, objSize);
  } else if (objSize >= 1 && ObjId != front_vision_objects_->object[objSize-1].id) {
    sendObjectsUdp(simTime, simFrame, objSize);
  }
}
void FrontVisionModel::tlightsHandler(const double &simTime,
                                      const unsigned int &simFrame,
                                      RDB_TRAFFIC_LIGHT_t &item,
                                      bool isExtended,
                                      bool isSensor,
                                      int tlSize) {
  if (isSensor && isExtended) {
    memcpy(&mLights, &item, sizeof(RDB_TRAFFIC_LIGHT_t));
    LOG(ERROR) << "红绿灯状态 = " << mLights.base.state;
    LOG(ERROR) << "红绿灯相位 = " << mLights.ext.duration;
    LOG(ERROR) << "红绿灯类型 = " << (uint32_t)mLights.ext.type;
  }
  uint32_t Id = mLights.base.id;
  if (tlSize == 0) {
    sendLightsUdp(simTime, simFrame, tlSize);
  } else if (tlSize >= 1 && Id != traffic_lights_->traffic_light[tlSize-1].id) {
    sendLightsUdp(simTime, simFrame, tlSize);
  }
}
void FrontVisionModel::trafficSignHandler(const double &simTime,
                                          const unsigned int &simFrame,
                                          RDB_TRAFFIC_SIGN_t &item,
                                          bool isSensor) {
  if (isSensor && item.type == 1000011)
    memcpy(&mSigns, &item, sizeof(RDB_TRAFFIC_SIGN_t));
  LOG(INFO) << "交通标志处理！！！" ;
  LOG(INFO) << "simTime = " << simTime <<  " , simFrame = " << simFrame;
  LOG(ERROR) << "交通灯类型(10->Left, 20->Right, 30->Straight) = " << mSigns.subType;
}
void FrontVisionModel::laneHandler(const double &simTime,
                                   const unsigned int &simFrame,
                                   RDB_ROADMARK_t &item,
                                   bool isSensor,
                                   int laneSize) {
  if (isSensor)
    memcpy(&mLaneLines, &item, sizeof(RDB_ROADMARK_t));
  LOG(INFO) << "Handle Sensor " << FRONT_SENSOR << " road info" ;
  LOG(INFO) << "simTime = " << simTime <<  " , simFrame = " << simFrame;
  LOG(INFO) << "lane_id = " << static_cast<int>(mLaneLines.laneId)
            << ", type = " << static_cast<int>(mLaneLines.type)
            << ", color = " << static_cast<int>(mLaneLines.color);
  int8_t laneID = mLaneLines.laneId;
  if (laneSize == 0) {
    sendLanesUdp(simTime, simFrame, laneSize);
  } else if (laneSize >= 1 &&  laneID != lane_lines_->lane[laneSize-1].laneId) {
    sendLanesUdp(simTime, simFrame, laneSize);
  }
}
void FrontVisionModel::roadHandler(const double &simTime,
                                   const unsigned int &simFrame,
                                   RDB_ROAD_POS_t &item,
                                   bool isSensor) {
  if (item.playerId == 1 && isSensor)
    memcpy(&mRoadPos, &item, sizeof(RDB_ROAD_POS_t));
}
void FrontVisionModel::sendObjectsUdp(const double &simTime, const unsigned int &simFrame, const int &objSize) {
  UdpSocketClient<FrontVisionObjects>::connectToUdpServer(LOCAL_HOST, obj_port);
  // Header
  front_vision_objects_->header.magicNo = RDB_MAGIC_NO;
  front_vision_objects_->header.version = RDB_VERSION;
  front_vision_objects_->header.headerSize = sizeof(FrontVisionObjects);
  front_vision_objects_->header.dataSize = sizeof(front_vision_objects_->object);
  front_vision_objects_->header.frameNo = simFrame;
  front_vision_objects_->header.simTime = simTime;
  if (mObjState.base.category == 1) {
    front_vision_objects_->object[objSize].id = mObjState.base.id;
    LOG(ERROR) << "障碍物ID= " << front_vision_objects_->object[objSize].id;
    front_vision_objects_->object[objSize].category = mObjState.base.category;
    front_vision_objects_->object[objSize].type = mObjState.base.type;
    front_vision_objects_->object[objSize].geo.dimX = mObjState.base.geo.dimX;
    front_vision_objects_->object[objSize].geo.dimY = mObjState.base.geo.dimY;
    front_vision_objects_->object[objSize].geo.dimZ = mObjState.base.geo.dimZ;
    front_vision_objects_->object[objSize].pos.x = mObjState.base.pos.x;
    front_vision_objects_->object[objSize].pos.y = mObjState.base.pos.y;
    front_vision_objects_->object[objSize].pos.z = mObjState.base.pos.z;
    LOG(ERROR) << "障碍物位置 x/y/z = " << mObjState.base.pos.x << "/"
               << mObjState.base.pos.y << "/" << mObjState.base.pos.z;
    front_vision_objects_->object[objSize].heading = mObjState.base.pos.h;
    front_vision_objects_->object[objSize].pitch   = mObjState.base.pos.p;
    front_vision_objects_->object[objSize].roll    = mObjState.base.pos.r;
    front_vision_objects_->object[objSize].speed.x = mObjState.ext.speed.x;// + mOwnObject.ext.speed.x;  //abs speed
    front_vision_objects_->object[objSize].speed.y = mObjState.ext.speed.y;// + mOwnObject.ext.speed.y;
    front_vision_objects_->object[objSize].speed.z = mObjState.ext.speed.z;// + mOwnObject.ext.speed.z;
    front_vision_objects_->object[objSize].accel.x = mObjState.ext.accel.x;// + mOwnObject.ext.accel.x; // abs accel
    front_vision_objects_->object[objSize].accel.y = mObjState.ext.accel.y;// + mOwnObject.ext.accel.y;
    front_vision_objects_->object[objSize].accel.z = mObjState.ext.accel.z;// + mOwnObject.ext.accel.z;
  }
  UdpSocketClient<FrontVisionObjects>::sendData(*front_vision_objects_);
  UdpSocketClient<FrontVisionObjects>::closeSocket();
}
void FrontVisionModel::sendLightsUdp(const double &simTime, const unsigned int &simFrame, const int &lightSize) {
  UdpSocketClient<TrafficLights>::connectToUdpServer(LOCAL_HOST, light_port);
  traffic_lights_->header.simTime = simTime;
  traffic_lights_->header.frameNo = simFrame;
  traffic_lights_->traffic_light[lightSize].id = mLights.base.id;
  LOG(INFO) << "红绿灯id = " << traffic_lights_->traffic_light[lightSize].id;
  switch (mLights.base.stateMask) {
    case 0x100000:
      traffic_lights_->traffic_light[lightSize].color = 3;
      break;
    case 0x1000000:
      traffic_lights_->traffic_light[lightSize].color = 2;
      break;
    case 0x10000000:
      traffic_lights_->traffic_light[lightSize].color = 1;
      break;
    default:
      traffic_lights_->traffic_light[lightSize].color = 0;
      break;
  }
  traffic_lights_->traffic_light[lightSize].show_type = mSigns.subType;
  LOG(ERROR) << "按照绿->黄->红颜色变化！";
  int32_t remaining_time;
  float state = mLights.base.state;
  float duration = mLights.ext.duration;
  switch (mLights.base.stateMask) {
    case 0x100000:
      remaining_time = (duration - state) * mLights.ext.cycleTime;
      break;
    case 0x1000000:
      remaining_time = 3;
      break;
    case 0x10000000:
      remaining_time = (1 - state) * mLights.ext.cycleTime;
      break;
    default:
      remaining_time = 0;
      break;
  }
  traffic_lights_->traffic_light[lightSize].remaining_time = remaining_time;
  LOG(ERROR) << "红绿灯颜色：" << (int)traffic_lights_->traffic_light[lightSize].color
             << ", 倒计时：" << remaining_time << "s!";
  UdpSocketClient<TrafficLights>::sendData(*traffic_lights_);
  UdpSocketClient<TrafficLights>::closeSocket();
}
void FrontVisionModel::sendLanesUdp(const double &simTime, const unsigned int &simFrame, const int &laneSize) {
  UdpSocketClient<LaneLines>::connectToUdpServer(LOCAL_HOST, lane_port);
  // lane line info
  lane_lines_->header.simTime = simTime;
  lane_lines_->header.frameNo =  simFrame;
  lane_lines_->lane[laneSize].laneId = mLaneLines.laneId;
  LOG(ERROR) << "车道线ID= " << (int)lane_lines_->lane[laneSize].laneId;
  lane_lines_->lane[laneSize].color  = mLaneLines.color;
  lane_lines_->lane[laneSize].type   = mLaneLines.type;
  lane_lines_->lane[laneSize].noDataPoints = mLaneLines.noDataPoints;
  lane_lines_->lane_size = laneSize;
  LOG(INFO) << "车道线数量 = " << laneSize;
  // current lane
  lane_lines_->lane[laneSize].curLaneId = mRoadPos.laneId;
  LOG(ERROR) << "本车车道ID= " << (int)mRoadPos.laneId;
  // lane poly
  LOG(INFO) << "视觉车道线拟合！！！";
  std::vector<double> points_x, points_y;
  for (int index = 0; index < mLaneLines.noDataPoints; index = index + 4) {
    if (mLaneLines.points == nullptr) {
      LOG(ERROR) << "车道线散点为空！！！";
      continue;
    } else {
      points_x.push_back(mLaneLines.points[index].x);
      points_y.push_back(mLaneLines.points[index].y);
//      LOG(ERROR) << "视觉车道线散点, x/y = " << mLaneLines.points[index].x << "/" << mLaneLines.points[index].y;
    }
  }
  if (!points_x.empty() && !points_y.empty()) {
    Eigen::VectorXd ret = lane_boundary_->FitUseRANSAC(3, points_x, points_y);
    lane_lines_->lane[laneSize].poly.start_x = mLaneLines.startDx;
    lane_lines_->lane[laneSize].poly.end_x = mLaneLines.previewDx;
    lane_lines_->lane[laneSize].poly.c0 = float(ret[0]);
    lane_lines_->lane[laneSize].poly.c1 = float(ret[1]);
    lane_lines_->lane[laneSize].poly.c2 = float(ret[2]);
    lane_lines_->lane[laneSize].poly.c3 = float(ret[3]);
    LOG(ERROR) << "start_x = " <<  lane_lines_->lane[laneSize].poly.start_x
               << ", poly coefficient c0/c1/c2/c3 = "
               << ret[0] << "/" << ret[1] << "/" << ret[2] << "/" << ret[3];
  }

  UdpSocketClient<LaneLines>::sendData(*lane_lines_);
  UdpSocketClient<LaneLines>::closeSocket();
}
void FrontVisionModel::run() {
  LOG(INFO) << "register [" << this->displayName().c_str() << "] module";
  static bool sVerbose = false;
  //init yaml file
  init();
  // initialize the connections
  initConnections();
//  memset(&mLaneLines, 0, sizeof(RDB_ROADMARK_t));
//  memset(&mObjState, 0, sizeof(RDB_OBJECT_STATE_t));
//  memset(&mRoadPos, 0, sizeof(RDB_ROAD_POS_t));
//  memset(&mLights, 0, sizeof(RDB_TRAFFIC_LIGHT_t));
  // open sensor port
  if (!openPort(sConnection[1].desc, sConnection[1].port, sConnection[1].serverAddr))
    return;
  // receive data
  readConnection(sConnection[1], false, sVerbose);
}
void FrontVisionModel::init() {
  char* install_root_path = getenv("SIM_INSTALL_ROOT_PATH");
  if (install_root_path == nullptr)
    LOG(ERROR) << "the environment variable 'SIM_INSTALL_ROOT_PATH' has not been set.";
  std::string config_path = std::string(install_root_path) + "/config/simulator/sensor_cfg.yaml";
  YAML::Node config_node;
  try {
    config_node = YAML::LoadFile(config_path);
  } catch (const YAML::BadFile& e) {
    LOG(ERROR) << "load conf yaml error" << config_path.c_str();
    exit(-1);
  }
  auto sensor_node = config_node["sensor"];
  sensor_ip = sensor_node["front_sensor"]["ip"].as<std::string>();
  sensor_port = sensor_node["front_sensor"]["port"].as<int>();
  auto rdb_node = config_node["rdb"];
  obj_port = rdb_node["obstacle"]["port"].as<int>();
  light_port = rdb_node["light"]["port"].as<int>();
  lane_port = rdb_node["lane"]["port"].as<int>();
}
} // namespace stoic::simulator
