aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} EGO_STATES_SRCS)

add_executable(vtd_ego ${EGO_STATES_SRCS})
target_link_libraries(vtd_ego
        vtd_dependencies
        ${YAML_CPP_LIBRARIES})

install(TARGETS vtd_ego
        RUNTIME
        DESTINATION bin)
