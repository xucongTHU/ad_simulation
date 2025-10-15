cmake_minimum_required(VERSION 3.16)
find_package(Protobuf REQUIRED)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

add_executable(img_save_video main.cpp)
target_link_libraries(img_save_video 
	communication cm_proto cm_art_core art_gpujpeg
)

install(TARGETS  img_save_video RUNTIME DESTINATION bin)
