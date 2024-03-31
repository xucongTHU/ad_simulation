
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} PARK_SRCS)
add_library(park_freespace_libs SHARED ${PARK_SRCS})
target_link_libraries(park_freespace_libs
        vtd_dependencies
        ${OpenCV_LIBS})
install(TARGETS park_freespace_libs
        LIBRARY
        DESTINATION lib)
