file(GLOB_RECURSE SIM_MSF_LOC_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/app/*.cpp)

add_library(sim_msf_lib SHARED
        ${SIM_MSF_LOC_SRCS})

target_link_libraries(sim_msf_lib ${STOIC_APP_LIBS})

install(TARGETS sim_msf_lib
        LIBRARY DESTINATION lib
        )

add_executable(sim_msf_loc main.cc)
target_link_libraries(sim_msf_loc sim_msf_lib)
install(TARGETS sim_msf_loc
        RUNTIME DESTINATION bin)
