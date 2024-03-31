file(GLOB_RECURSE BRIDGE_DEPEND_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp)

add_library(bridge_dependencies SHARED
            ${BRIDGE_DEPEND_SRCS})

target_link_libraries(bridge_dependencies
                      ${PROTOBUF_LIBRARIES}
)

install(TARGETS bridge_dependencies
        LIBRARY DESTINATION lib
)