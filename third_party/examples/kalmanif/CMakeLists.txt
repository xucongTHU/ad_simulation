add_executable(demo_se2 demo_se2.cpp)
add_executable(demo_se3 demo_se3.cpp)
add_executable(demo_se_2_3 demo_se_2_3.cpp)

target_link_libraries(demo_se2 ${catkin_LIBRARIES})
target_link_libraries(demo_se3 ${catkin_LIBRARIES})
target_link_libraries(demo_se_2_3 ${catkin_LIBRARIES})

install(TARGETS demo_se2 demo_se3 demo_se_2_3
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

