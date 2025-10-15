cmake_minimum_required(VERSION ${CMAKE_VERSION})

project(ad_simulation)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/submodule)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/submodule)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/submodule/third_party)

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
add_subdirectory(submodule/third_party)

install(DIRECTORY  scripts/art/
        DESTINATION scripts
)

#############
##  CPack  ##
#############

set(CPACK_GENERATOR "TGZ")

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VERSION ${MODULE_VERSION})
set(CPACK_PACKAGE_CONTACT "xucongTHU@163.com")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)
set(CPACK_SET_DESTDIR ON)
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(CPACK_PACKAGE_FILE_NAME "${MODULE_VERSION}")

install(
        FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/resource/tar_extra/release_helper.sh
        DESTINATION /
)

set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
include(CPack)