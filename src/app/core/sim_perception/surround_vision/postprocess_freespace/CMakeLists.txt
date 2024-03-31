
add_library(freespace_process SHARED freespace_process.cpp)
target_link_libraries(freespace_process
        common
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )
install(TARGETS freespace_process LIBRARY DESTINATION lib)
