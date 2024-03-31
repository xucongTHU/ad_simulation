file(GLOB SRC_FILES "./src/*.c" "./src/*.cpp")
file(GLOB CU_FILES "./src/*.cu")

set(CMAKE_CXX_FLAGS "-std=c++14 -fext-numeric-literals -pipe -O2 -Wall -Wextra -fopenmp -fPIC -pthread -Wdeprecated-declarations")

find_package(CUDA)
include_directories(${CUDA_INCLUDE_DIRS})

#message("333333333333333333" ${CUDA_LIBRARIES})

file(GLOB CUDA_LIBS ${CUDA_LIBRARIES}/*.so)

#message("33333333333333333344444444444444" ${CUDA_LIBS})

include_directories(./include ./)

cuda_add_library(art_cuda_gpujpeg SHARED ${CU_FILES})
target_link_libraries(art_cuda_gpujpeg ${CUDA_LIBS})
install(TARGETS art_cuda_gpujpeg LIBRARY DESTINATION lib)

add_library(art_gpujpeg SHARED
            gpu_codec.cpp
            ${SRC_FILES}
)

target_link_libraries(art_gpujpeg
                      art_cuda_gpujpeg
                      ${CUDA_LIBS}
                      )
install(TARGETS art_gpujpeg LIBRARY DESTINATION lib)
