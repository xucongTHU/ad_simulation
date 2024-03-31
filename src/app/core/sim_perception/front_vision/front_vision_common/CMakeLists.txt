
add_library(frontvision_common SHARED frontvision_det_alg.cpp)
target_link_libraries(frontvision_common
        common
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )
install(TARGETS frontvision_common LIBRARY DESTINATION lib)
