
add_library(parkspace_process SHARED parkingspace_tracker.cpp)
target_link_libraries(parkspace_process
        common
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )
install(TARGETS parkspace_process LIBRARY DESTINATION lib)
