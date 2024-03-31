add_library(lane_type SHARED
            cubic_polynomial.cc
            lane_boundary.cc
            lane_fuision_data_type.h
            lane_fusion_debugger.h
            lane_fusion_dependency.h
            lane.cc
            point.h
)

target_link_libraries(lane_type
                      # lane_fusion_kf
                      # lane_fusion_lane_model
                      # lane_fusion_lane_source_manager
                      # planning_extended_lane_source
                      # ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
)

if(CMAKE_MW STREQUAL "ROS")
  add_dependencies(lane_type ros_custom_mes_generate_messages_cpp)
endif()

# add_subdirectory(lane_source_manager)
# add_subdirectory(kf)
# add_subdirectory(lane_model)


# add_subdirectory(extended_lane_source)
install(TARGETS lane_type LIBRARY DESTINATION lib)
