aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/app VISION_SRCS)

add_library(front_vision_libs SHARED ${VISION_SRCS})
target_link_libraries(front_vision_libs
        PRIVATE lane_type
        PRIVATE vtd_dependencies
        ${OpenCV_LIBS})

install(TARGETS front_vision_libs
        LIBRARY
        DESTINATION lib)

add_subdirectory(lane_line_type)
