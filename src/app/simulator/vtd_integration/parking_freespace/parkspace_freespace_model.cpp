#include "simulator/vtd_integration/parkingspace_freespace/parkspace_freespace_model.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace stoic::simulator {
int ParkFreespaceModel::openPort(int &descriptor, int portNo, const char *serverAddr) {
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
void ParkFreespaceModel::initConnections() {
  memset( sConnection, 0, MAX_CONNECTIONS * sizeof(Connection_t));
  for ( int i = 0; i < MAX_CONNECTIONS; i++) {
    initConnection(sConnection[i]);
    sConnection[i].id = i;
  }
  sConnection[0].port = DEFAULT_PORT_TC;
  sConnection[1].port = sensor_port;
}
void ParkFreespaceModel::initConnection(Connection_t &conn) {
  strcpy(conn.serverAddr, sensor_ip.c_str());
  conn.desc = -1;
  conn.bufferSize = sizeof(RDB_MSG_t);
  conn.pData = (unsigned char*) calloc(1, conn.bufferSize);
}
void ParkFreespaceModel::readConnection(Connection_t &conn, bool waitForMessage, bool verbose) {
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
void ParkFreespaceModel::parseRDBMessage(Connection_t &conn, RDB_MSG_t *msg) {
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
void ParkFreespaceModel::parseRDBMessageEntry(Connection_t &conn,
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
          memset(&mParkspaceState, 0, sizeof(RDB_OBJECT_STATE_t));
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
  int parking_space_size = 0;
//  unsigned char ident = 6;
  char* dataPtr = (char*) entryHdr;
  dataPtr += entryHdr->headerSize;
  int noParkspace= sizeof(parking_spaces_->parking_space) / sizeof(sim_ground_truth::ParkingSpace);
  while (noElements-- && parking_space_size < noParkspace) {
    bool printedMsg = true;
    auto parkspacePtr   = (RDB_OBJECT_STATE_t*)dataPtr;
    switch (entryHdr->pkgId)
    {
      case RDB_PKG_ID_OBJECT_STATE:
        if (conn.id == 1) {
          handleParkingSpace(simTime, simFrame, *parkspacePtr, entryHdr->flags & RDB_PKG_FLAG_EXTENDED, conn.id == 1u, parking_space_size);
          parking_space_size++;
        }
        break;
      default:
        printedMsg = false;
        break;
    }
    dataPtr += entryHdr->elementSize;
  }
}
void ParkFreespaceModel::handleParkingSpace(const double &simTime,
                                            const unsigned int &simFrame,
                                            RDB_OBJECT_STATE_t &item,
                                            bool isExtended,
                                            bool isSensor,
                                            int parking_space_size) {
  memcpy(&mSurVisionState, &item, sizeof(RDB_OBJECT_STATE_t));
  if (item.base.type == RDB_OBJECT_TYPE_PARKING_SPACE)
    memcpy(&mParkspaceState, &item, sizeof(RDB_OBJECT_STATE_t));
  LOG(INFO) << "Handle surround sensor " <<  SURROUND_SENSOR << " info";
  LOG(INFO) << "simTime = " << simTime <<  " , simFrame = " << simFrame;
  uint32_t parkId = mParkspaceState.base.id;
  if (parking_space_size == 0) {
    sendParkingspace(simTime, simFrame, parking_space_size);
  } else if (parking_space_size >= 1 && parkId != parking_spaces_->parking_space[parking_space_size-1].id) {
    sendParkingspace(simTime, simFrame, parking_space_size);
  }
}
void ParkFreespaceModel::sendParkingspace(const double &simTime,
                                          const unsigned int &simFrame,
                                          const int &parking_space_size) {
  UdpSocketClient<ParkingSpaces>::connectToUdpServer(LOCAL_HOST, parkspace_port);
  // Header
  parking_spaces_->header.magicNo = RDB_MAGIC_NO;
  parking_spaces_->header.version = RDB_VERSION;
  parking_spaces_->header.headerSize = sizeof(ParkingSpaces);
  parking_spaces_->header.dataSize = sizeof(parking_spaces_->parking_space);
  parking_spaces_->header.frameNo = simFrame;
  parking_spaces_->header.simTime = simTime;
  double xo = mParkspaceState.base.pos.x;
  double yo = mParkspaceState.base.pos.y;
  double h = mParkspaceState.base.pos.h;
  double geo_x = mParkspaceState.base.geo.dimX;
  double geo_y = mParkspaceState.base.geo.dimY;
  double geo_z = mParkspaceState.base.geo.dimZ;
  Eigen::Matrix3d TX_V_M;
  TX_V_M << xo, 1.0, 1.0, yo, 1.0, 1.0, 0.0, 0.0, 1.0;
  LOG(ERROR) << "车位矩阵为=\n" << TX_V_M;
  Eigen::Matrix3d matrix1;
  Eigen::Vector3d I_wi(1, 1, 0);
  matrix1.row(0) = I_wi.transpose();
  matrix1(1,0) = (geo_x/2+geo_z) * std::cos(h);
  matrix1(1,1) = (geo_x/2+geo_z) * std::sin(h);
  matrix1(1,2) = 0;
  matrix1(2, 0) = -geo_y / 2 * std::sin(h);
  matrix1(2, 1) = geo_y / 2 * std::cos(h);
  matrix1(2, 2) = 1;
  Eigen::Matrix3d matrix2;
  matrix2.row(0) = I_wi.transpose();
  matrix2(1,0) = -(geo_x/2-geo_z) * std::cos(h);
  matrix2(1,1) = -(geo_x/2-geo_z) * std::sin(h);
  matrix2(1,2) = 0;
  matrix2(2, 0) = -geo_y / 2 * std::sin(h);
  matrix2(2, 1) = geo_y / 2 * std::cos(h);
  matrix2(2, 2) = 1;
  Eigen::Matrix3d matrix3;
  matrix3.row(0) = I_wi.transpose();
  matrix3(1,0) = -(geo_x/2-geo_z) * std::cos(h);
  matrix3(1,1) = -(geo_x/2-geo_z) * std::sin(h);
  matrix3(1,2) = 0;
  matrix3(2, 0) = geo_y / 2 * std::sin(h);
  matrix3(2, 1) = -geo_y / 2 * std::cos(h);
  matrix3(2, 2) = 1;
  Eigen::Matrix3d matrix4;
  matrix4.row(0) = I_wi.transpose();
  matrix4(1,0) = (geo_x/2+geo_z) * std::cos(h);
  matrix4(1,1) = (geo_x/2+geo_z) * std::sin(h);
  matrix4(1,2) = 0;
  matrix4(2, 0) = geo_y / 2 * std::sin(h);
  matrix4(2, 1) = -geo_y / 2 * std::cos(h);
  matrix4(2, 2) = 1;
  //按照逆时针A1->A2->A3->A4计算角点坐标
  Eigen::Matrix3d A1 = TX_V_M * matrix1;
  Eigen::Matrix3d A2 = TX_V_M * matrix2;
  Eigen::Matrix3d A3 = TX_V_M * matrix3;
  Eigen::Matrix3d A4 = TX_V_M * matrix4;
  
  std::string parkspace_name = mParkspaceState.base.name;
  parking_spaces_->parking_space[parking_space_size].type = RDB_OBJECT_TYPE_PARKING_SPACE;
  if (strcmp(parkspace_name.c_str(), "RdParkingBox_00Deg_White.flt") == 0) {
    LOG(INFO) << "检测车位是水平车位！！！";
    parking_spaces_->parking_space[parking_space_size].id = mParkspaceState.base.id;
    LOG(INFO) << "[ParkFreeSpaceModel] 计算车位四个角点 ...";
    parking_spaces_->parking_space[parking_space_size].park_left_up_point.x = A1(1);
    parking_spaces_->parking_space[parking_space_size].park_left_up_point.y = A1(5);
    parking_spaces_->parking_space[parking_space_size].park_left_down_point.x = A4(1);
    parking_spaces_->parking_space[parking_space_size].park_left_down_point.y = A4(5);
    parking_spaces_->parking_space[parking_space_size].park_right_up_point.x = A2(1);
    parking_spaces_->parking_space[parking_space_size].park_right_up_point.y = A2(5);
    parking_spaces_->parking_space[parking_space_size].park_right_down_point.x = A3(1);
    parking_spaces_->parking_space[parking_space_size].park_right_down_point.y = A3(5);
    LOG(ERROR) << "[ParkFreeSpaceModel] 角点坐标为：\n"
               << "左上角点坐标(x2, y2) = (" << A1(1) << ", " << A1(5) << ")\n"
               << "左下角点坐标(x3, y3) = (" << A4(1) << ", " << A4(5) << ")\n"
               << "右上角点坐标(x1, y1) = (" << A2(1) << ", " << A2(5) << ")\n"
               << "右下角点坐标(x4, y4) = (" << A3(1) << ", " << A3(5) << ")";
    parking_spaces_->parking_space[parking_space_size].angleLeft = M_PI_2;
    parking_spaces_->parking_space[parking_space_size].angleRight = M_PI_2;
    parking_spaces_->parking_space[parking_space_size].park_length = geo_x;
    parking_spaces_->parking_space[parking_space_size].park_width = geo_y;
  } else if (strcmp(parkspace_name.c_str(), "RdParkingBox_90Deg_White.flt") == 0) {
    LOG(INFO) << "检测车位是垂直车位！！！";
    parking_spaces_->parking_space[parking_space_size].park_right_down_point.x = A2(1);
    parking_spaces_->parking_space[parking_space_size].park_right_down_point.y = A2(5);
    parking_spaces_->parking_space[parking_space_size].park_right_up_point.x = A1(1);
    parking_spaces_->parking_space[parking_space_size].park_right_up_point.y = A1(5);
    parking_spaces_->parking_space[parking_space_size].park_left_down_point.x = A3(1);
    parking_spaces_->parking_space[parking_space_size].park_left_down_point.y = A3(5);
    parking_spaces_->parking_space[parking_space_size].park_left_up_point.x = A4(1);
    parking_spaces_->parking_space[parking_space_size].park_left_up_point.y = A4(5);
    LOG(ERROR) << "[ParkFreeSpaceModel] 角点坐标为：\n"
               << "右下角点坐标(x4, y4) = (" << A2(1) << ", " << A2(5) << ")\n"
               << "右上角点坐标(x1, y1) = (" << A1(1) << ", " << A1(5) << ")\n"
               << "左下角点坐标(x3, y3) = (" << A3(1) << ", " << A3(5) << ")\n"
               << "左上角点坐标(x2, y2) = (" << A4(1) << ", " << A4(5) << ")";
    parking_spaces_->parking_space[parking_space_size].angleLeft = M_PI_2;
    parking_spaces_->parking_space[parking_space_size].angleRight = M_PI_2;
    parking_spaces_->parking_space[parking_space_size].park_length = geo_x;
    parking_spaces_->parking_space[parking_space_size].park_width = geo_y;
  } else {
    LOG(ERROR) << "未检测车位！！！";
    return;
  }
  UdpSocketClient<ParkingSpaces>::sendData(*parking_spaces_);
  UdpSocketClient<ParkingSpaces>::closeSocket();
}
void ParkFreespaceModel::run() {
  LOG(INFO) << "register [" << this->displayName().c_str() << "] module";
  static bool sVerbose = false;
  //init yaml file
  init();
  // initialize the connections
  initConnections();
  // open sensor port
  if (!openPort(sConnection[1].desc, sConnection[1].port, sConnection[1].serverAddr))
    return;
  // receive data
  readConnection(sConnection[1], false, sVerbose);
}
void ParkFreespaceModel::init() {
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
  sensor_ip = sensor_node["frontfisheye_sensor"]["ip"].as<std::string>();
  sensor_port = sensor_node["frontfisheye_sensor"]["port"].as<int>();
  auto rdb_node = config_node["rdb"];
  parkspace_port = rdb_node["parking"]["port"].as<int>();
}
} // namespace stoic::simulator
