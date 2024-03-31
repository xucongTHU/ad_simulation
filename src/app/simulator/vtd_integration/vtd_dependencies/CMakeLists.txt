file(GLOB_RECURSE VTD_COMMON_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp)

add_library(vtd_dependencies SHARED
        ${VTD_COMMON_SRCS})

target_link_libraries(vtd_dependencies
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

install(TARGETS vtd_dependencies
        LIBRARY DESTINATION lib
        )
