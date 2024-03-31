cmake_minimum_required(VERSION ${CMAKE_VERSION})

project(stoicheia)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/submodule)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/submodule)

#map
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/submodule/map)
    message("======compile with map======")
    add_compile_definitions(SD_MAP)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
        file(GLOB MAP_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/submodule/map/lib/x86_64/*.*)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
        file(GLOB MAP_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/submodule/map/lib/aarch64/*.*)
    endif()
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/submodule/map/include)
else()
    message("======compile without map======")
endif()

#interface
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/submodule/interface/include)

add_subdirectory(src)

install(DIRECTORY  scripts/art/
        DESTINATION scripts
)