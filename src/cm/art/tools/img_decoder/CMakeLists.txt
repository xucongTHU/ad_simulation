cmake_minimum_required(VERSION 3.16)
find_package(Protobuf REQUIRED)

add_executable(stoic_image_decoder main.cpp image_decoder.cpp camera_driver.cpp camera_loader.cpp)
target_link_libraries(stoic_image_decoder 
	communication cm_proto cm_art_core art_gpujpeg
	# opencv_core
	# opencv_imgproc
	# opencv_highgui
	# opencv_imgcodecs
	# opencv_calib3d
	# opencv_videoio
)

install(TARGETS  stoic_image_decoder RUNTIME DESTINATION bin)
