
cmake_minimum_required(VERSION ${CMAKE_VERSION})
project(simulation)
#common package
message(STATUS  "Step1: ------Find app dependencies with common packages------")
find_package(Boost REQUIRED COMPONENTS filesystem system iostreams thread)
include_directories(${Boost_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#perc
option(ENABLE_LANE_MODEL "perception lane model" ON)
if(ENABLE_LANE_MODEL)
  add_compile_definitions(LANE_LINE_MODEL)
endif()
option(ENABLE_BEV_MODEL "perception bev model" OFF)
if(ENABLE_BEV_MODEL)
  add_compile_definitions(BEV_VISION_MODEL)
endif()
#cm
message(STATUS "Step2: ------CM Adapter------")
if(CMAKE_MW STREQUAL  "AP")
  message(STATUS "------Adaptive AutoSAR MiddleWare------")
  set(ACL_LIB_PATH /home/HwHiAiUser/Ascend/ascend-toolkit/5.0.mdc610/acllib_linux1.1.aarch64/acllib/lib64/stub) #TODO
  set(ACL_HAL_LIB_PATH /opt/t3caic/stoic/platform/mdc_aarch64_linux/mdc_sdk/dp_gea/mdc_cross_compiler/sysroot/usr/local/Ascend/runtime/lib64/stub) #TODO
  link_directories(${ACL_LIB_PATH})
  link_directories(${ACL_HAL_LIB_PATH})
  link_directories(
    ${MDC_SDK}/sysroot/usr/local/Ascend/runtime/lib64
    ${MDC_SDK}/sysroot/usr/local/Ascend/runtime/lib64/stub
  )
  link_directories(${MDC_SDK}/sysroot/usr/lib
    ${MDC_SDK}/sysroot/usr/lib/mdc_vector
    ${MDC_SDK}/sysroot/usr/lib/mdc/base-plat
    ${MDC_ACLLIB_SDK}/lib64/stub
    ${MDC_ACLLIB_SDK}/lib6
  )
elseif(CMAKE_MW STREQUAL "ART")
  message(STATUS "------Apollo CyberRT MiddleWare------")
  # SOME/IP
#  find_package(CommonAPI-SomeIP REQUIRED)
#  find_package(vsomeip3 REQUIRED)
  include_directories(
    ${COMMONAPI_INCLUDE_DIRS}
    ${COMMONAPI_SOMEIP_INCLUDE_DIRS}
    ${VSOMEIP_INCLUDE_DIRS}
  )
  link_directories(
    ${COMMONAPI_LIBDIR}
    ${COMMONAPI_SOMEIP_CMAKE_DIR}/build
  )
elseif(CMAKE_MW STREQUAL  "ROSM")
  message(STATUS "------ROS automessage MiddleWare-----")
  include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../
  )
  target_precompile_headers(simulation
      PRIVATE ${MSG_CORE_HEADER}
  )
  target_include_directories(simulation
      PRIVATE ${CATKIN_DEVEL_PREFIX}
      PRIVATE ${CATKIN_DEVEL_PREFIX}/include
  )
  target_link_libraries(simulation
      PRIVATE common
      PRIVATE cm_proto
      PRIVATE communication
      PRIVATE ${catkin_LIBRARIES}
      PRIVATE ${Boost_LIBRARIES}
      PRIVATE ${YAML_CPP_LIBRARIES}
  )
  install(TARGETS simulation
          RUNTIME
          DESTINATION bin
  )
  return()
endif()

#lib dependencies
message(STATUS "Step3: ------Add App SRCS and LIBS------")
if(CMAKE_MW STREQUAL "AP")
  set(SRCS
    ${SRCS_BRIDGE} ${SRCS_CORE}
  )
  set(SIM_APP_LIBS
          common
          communication
          cm_proto
          ${GFLAGS_LIBRARIES}
          ${GLOG_LIBRATIES}
          ${Boost_LIBRARIES}
          ${YAML_CPP_LIBRARIES}
  )
elseif(CMAKE_MW STREQUAL "ART")
  set(CM_ART
          ##添加cm-art库
          cm_art_base
          cm_art_blocker
          cm_art_class_loader
          cm_art_common
          cm_art_component
          cm_art_core
          cm_art_croutine
          cm_art_event
          cm_art_io
          cm_art_logger
          cm_art_message
          cm_art_node
          cm_art_parameter
          cm_cyber_proto
          cm_art_record
          cm_art_scheduler
          cm_art_service_discovery
          cm_art_sysmo
          cm_art_task
          cm_art_time
          cm_art_timer
          cm_art_transport)
  set(SIM_APP_LIBS
          common
          communication
          cm_proto
          ${CM_ART}
          ${MAP_LIBS}
          ${GFLAGS_LIBRARIES}
          ${GLOG_LIBRATIES}
          ${Boost_LIBRARIES}
          ${YAML_CPP_LIBRARIES}
  )
elseif(CMAKE_MW STREQUAL "ROS")
  set(SIM_APP_LIBS
          common
          communication
          cm_proto
          ${GFLAGS_LIBRARIES}
          ${GLOG_LIBRATIES}
          ${Boost_LIBRARIES}
          ${YAML_CPP_LIBRARIES}
  )
endif()
message(STATUS "Step4: ------multiple process------")
add_subdirectory(bridge)
add_subdirectory(core)
add_subdirectory(test)
add_executable(simulation simulation.cpp)
target_link_libraries(simulation bridge_app core_app)
install(TARGETS simulation
        RUNTIME DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DESTINATION config
        FILES_MATCHING PATTERN "*.yaml" PATTERN "*.pb.txt" PATTERN "*.cfg")
