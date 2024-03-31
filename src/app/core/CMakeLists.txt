
cmake_minimum_required(VERSION 3.20.5)
project(core_pipelines
        DESCRIPTION "core app pipeline entry point")
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/sim_canbus CAN_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/sim_msf_localization/app LOC_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/sim_perception/front_vision FRONT_VIS_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/sim_perception/surround_vision SURROUND_VIS_SRCS)
set(SRCS ${CAN_SRCS} ${LOC_SRCS} ${FRONT_VIS_SRCS} ${SURROUND_VIS_SRCS})
add_library(core_app SHARED ${SRCS})
target_link_libraries(core_app
        PRIVATE frontvision_common
        PUBLIC parkspace_process
        PUBLIC freespace_process
        PRIVATE core_dependencies
        PRIVATE communication
        PRIVATE cm_proto
        PUBLIC apollo
        PUBLIC common
        ${CM_ART}
        ${MAP_LIBS}
        ${GFLAGS_LIBRARIES}
        ${GLOG_LIBRATIES}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${OpenCV_LIBRARIES})
if(CMAKE_MW STREQUAL "ART")
    set(CORE_LIBS core_app)
endif()
add_subdirectory(core_dependencies)
add_subdirectory(sim_perception)
install(TARGETS core_app
        LIBRARY DESTINATION lib)
