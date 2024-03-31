# # find_package(Boost REQUIRED COMPONENTS filesystem system iostreams thread)
# # include_directories(${Boost_INCLUDE_DIRS})

# # find_package(yaml-cpp REQUIRED)
# # include_directories(${YAML_INCLUDE_DIRS})

# # include_directories(${CMAKE_SYSROOT}/usr/include)

# # add_executable(stoic_image_encoder ./main.cpp ./image_encoder.cpp ./image_encoder_driver.cpp )
# add_executable(stoic_image_encoder ./main.cpp)
# target_link_libraries(stoic_image_encoder
#                     PRIVATE  communication cm_proto  gflags cm_art_core
#                     protobuf pthread   common  ${GLOG_LIBRARIES}
#                     # PRIVATE opencv_core
#                     # PRIVATE opencv_imgproc
#                     # PRIVATE opencv_highgui
#                     # PRIVATE opencv_imgcodecs
#                     # PRIVATE opencv_calib3d
#                     # PRIVATE opencv_videoio
# )

# install(TARGETS  stoic_image_encoder RUNTIME DESTINATION bin)
cmake_minimum_required(VERSION 3.16)
find_package(Protobuf REQUIRED)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

add_executable(stoic_image_encoder main.cpp image_encoder.cpp image_encoder_driver.cpp camera_loader.cpp)
target_link_libraries(stoic_image_encoder 
	communication cm_proto cm_art_core art_gpujpeg
	# opencv_core
	# opencv_imgproc
	# opencv_highgui
	# opencv_imgcodecs
	# opencv_calib3d
	# opencv_videoio
)

install(TARGETS  stoic_image_encoder RUNTIME DESTINATION bin)
