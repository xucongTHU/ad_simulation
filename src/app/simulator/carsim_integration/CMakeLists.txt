aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} CARSIM_SRCS)
add_executable(carsim_stoic ${CARSIM_SRCS})
target_link_libraries(carsim_stoic
                        carsim_calc
                        )

install(TARGETS carsim_stoic
        RUNTIME DESTINATION bin)

add_subdirectory(vehicle_states_calc)
add_subdirectory(carsim_dependencies)
