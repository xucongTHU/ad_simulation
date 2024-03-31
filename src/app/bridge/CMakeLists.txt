
cmake_minimum_required(VERSION 3.20.5)
project(bridge DESCRIPTION "bridge app pipeline entry point")
option(RUN_WITH_MAP "run simulation without perc module" OFF)
if (RUN_WITH_MAP)
    add_compile_definitions(RUN_WITH_MAP)
endif()
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/driver DRI_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/sensor SENSE_SRC)
set(SRCS ${DRI_SRC} ${SENSE_SRC})
add_library(bridge_app SHARED ${SRCS})
target_link_libraries(bridge_app
        PRIVATE bridge_dependencies
        PRIVATE communication
        PRIVATE cm_proto
        PUBLIC common
        PUBLIC apollo
        ${CM_ART}
        ${GFLAGS_LIBRARIES}
        ${GLOG_LIBRATIES}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES})
if(CMAKE_MW STREQUAL "ART")
    set(BRIDGE_LIBS bridge_app)
endif()
add_subdirectory(bridge_dependencies)
install(TARGETS bridge_app
        LIBRARY DESTINATION lib)
