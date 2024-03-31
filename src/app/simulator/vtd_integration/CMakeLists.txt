cmake_minimum_required(VERSION 3.20.5)

project(vtd_stoic
        DESCRIPTION "vtd stoic entry point")

add_executable(vtd_stoic vtd_stoic.cc)
target_link_libraries(vtd_stoic
        PRIVATE bev_libs
        PRIVATE front_vision_libs
        common
        ${PROTOBUF_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${GLOG_LIBRATIES}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES})

add_subdirectory(vtd_dependencies)
#add_subdirectory(time_sync)
add_subdirectory(ego_states)
add_subdirectory(bev_model)
add_subdirectory(front_vision)

install(TARGETS vtd_stoic
        RUNTIME
        DESTINATION bin)
