
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDEDIR})
file(GLOB_RECURSE CORE_COMMON_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(core_dependencies SHARED
           ${CORE_COMMON_SRCS})
target_link_libraries(core_dependencies
        ${PROTOBUF_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        cm_proto
)
install(TARGETS core_dependencies
        LIBRARY DESTINATION lib
)
