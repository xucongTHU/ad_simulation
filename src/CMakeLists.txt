cmake_minimum_required(VERSION ${CMAKE_VERSION})

message(STATUS "Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

add_definitions(-w) #TODO

add_compile_definitions(STOIC_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

if(CMAKE_MW STREQUAL "ROS" AND CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
  set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals -pipe -O2 -mavx2 -mno-avx512f -Wall -Wextra -fopenmp -fPIC -pthread") 
elseif(CMAKE_MW STREQUAL "ROS" AND CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
  set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals -pipe -O2 -Wall -Wextra -fopenmp -fPIC -pthread")
elseif(CMAKE_MW STREQUAL "AP")
  set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals -pipe -O2 -Wall -Wextra -fopenmp -fPIC -pthread")
elseif(CMAKE_MW STREQUAL "RT")
  set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals -pipe -O2 -Wall -Wextra -fopenmp -fPIC -pthread")
elseif(CMAKE_MW STREQUAL "ART" AND CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
  set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals -pipe -O2 -mavx2 -mno-avx512f -Wall -Wextra -fopenmp -fPIC -pthread") 
elseif(CMAKE_MW STREQUAL "ART" AND CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
  set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals -pipe -O2 -Wall -Wextra -fopenmp -fPIC -pthread")
else()
  set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals -pipe -O2 -mavx2 -mno-avx512f -Wall -Wextra -fopenmp -fPIC -pthread") 
endif()

add_link_options(-Wl,--disable-new-dtags) # use rpath instead of runpath

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
set(CMAKE_INSTALL_RPATH  "$ORIGIN/../lib")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
  ${CMAKE_INSTALL_PREFIX}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/app
  ${CMAKE_CURRENT_SOURCE_DIR}/interface/sim_interface/include
)

find_package(Protobuf 3 EXACT REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

find_library(GLOG_LIBRARIES glog)
find_path(GLOG_INCLUDE_DIRS "glog/logging.h")

if(CMAKE_BUILD_TYPE MATCHES Debug)
  add_compile_definitions(DEBUG)
endif()

if(CMAKE_MW STREQUAL "ROS")
  message(STATUS "USING ROS")
  find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge std_msgs message_generation)

  include_directories(${catkin_INCLUDE_DIRS})
  include_directories(${CMAKE_BINARY_DIR}/devel/include)

  set(CM_NAME "ros")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    add_compile_definitions(MW_ROS_IPC)
    set(PLATFORM_NAME "ipc")
    set(ARCH_CM "x86_ROS")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    add_compile_definitions(MW_ROS_ORIN)
    set(PLATFORM_NAME "orin")
    set(ARCH_CM "ARM_ROS")
  endif()

  set(TRT_HOME /opt/usr/t3caic/third_party/TensorRT-8)
  include_directories(${TRT_HOME}/include)

elseif(CMAKE_MW STREQUAL "AP")
  add_compile_definitions(MW_AP)
  if(CMAKE_MW_STRATEGY MATCHES 3L1V)
    add_compile_definitions(MW_AP_3L1V)
  endif()
  set(CM_NAME "ap")
  set(PLATFORM_NAME "mdc810")
  set(AP_INC_PATH ${MDC_SDK}/sysroot/usr/include)
  set(ACL_HAL_INC_PATH ${AP_INC_PATH}/driver)
  set(ACL_INC_PATH /home/HwHiAiUser/Ascend/ascend-toolkit/5.0.mdc610/acllib_linux1.1.aarch64/runtime/include)
  include_directories(${ACL_HAL_INC_PATH})
  include_directories(${ACL_INC_PATH})
  include_directories(
    ${ara-com_INCLUDE_DIRS}
    ${ara-core_INCLUDE_DIRS}
    ${ara-exec_INCLUDE_DIRS}
    ${MDC_SDK}/sysroot/usr/include/driver
    ${MDC_SDK}/sysroot/usr/include/adsfi/adb/include
    ${MDC_SDK}/sysroot/usr/include/opencv4
    ${MDC_ACLLIB_SDK}/include
    ${MDC_ACLLIB_SDK}/include/acl
    ${MDC_ACLLIB_SDK}/include/acl/ops
    ${MDC_ACLLIB_SDK}/include/acl/dvpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cm/ap/generated
  )

elseif(CMAKE_MW STREQUAL "RT")
  add_compile_definitions(MW_RT)
  set(CM_NAME "rt")
  set(PLATFORM_NAME "a1000")

  include_directories(${cyber_SOURCE_DIR})
  include_directories(${cyber_BINARY_DIR})

  find_package(Protobuf REQUIRED)
  if(PROTOBUF_FOUND)
    message(STATUS "Found Protobuf ${PROTOBUF_INCLUDE_DIR} ${PROTOBUF_LIBRARIES}")
  else()
    message(FATAL_ERROR "protobuf Not Found")
  endif(PROTOBUF_FOUND)

  # SOME/IP
  find_package(CommonAPI-SomeIP REQUIRED)
  find_package(vsomeip3 REQUIRED)

  include_directories(
    ${COMMONAPI_INCLUDE_DIRS}
    ${COMMONAPI_SOMEIP_INCLUDE_DIRS}
    ${VSOMEIP_INCLUDE_DIRS}
  )

  link_directories(
    ${COMMONAPI_LIBDIR}
    ${COMMONAPI_SOMEIP_CMAKE_DIR}/build
  )
  add_compile_options(-O2)

elseif(CMAKE_MW STREQUAL "ART")
  message(STATUS "USING ART(Apollo-CyberRT)")

  set(CM_NAME "art")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    add_compile_definitions(MW_ART_IPC)
    set(PLATFORM_NAME "ipc")
    set(ARCH_CM "x86_ART")
  endif()
  if(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    add_compile_definitions(MW_ART_ORIN)
    set(PLATFORM_NAME "orin")
    set(ARCH_CM "ARM_ART")
    set(CUDA_HOME /usr/local/cuda)
    set(CUDA_TOOLKIT_ROOT_DIR ${CUDA_HOME})
    set(CUDA_CUDART_LIBRARY ${CUDA_HOME}/lib64)
    set(CUDA_cublas_LIBRARY ${CUDA_HOME}/lib64)
    set(CUDA_curand_LIBRARY ${CUDA_HOME}/lib64)
    unset(CUDART CACHE)
    find_library(CUDART cudart ${CUDA_HOME}/lib64)
    set(TRT_LIB_PATH ${TRT_HOME}/targets/aarch64-linux-gnu/lib)
    include_directories(/usr/include/opencv4)
  endif()

  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  find_package(CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})

  ### Find TensorRT
  set(TRT_HOME /opt/usr/t3caic/third_party/TensorRT-8)
  include_directories(${TRT_HOME}/include)

  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cm/art)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cm/art/cyber)
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/cm/art)
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/cm/art/cyber)

  #  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../build/${ARCH_CM}/install/cm/include/src)
#  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../build/${ARCH_CM}/install/cm/include/src/cm)
#  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../build/${ARCH_CM}/install/cm/include/src/cm/art)
#  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../build/${ARCH_CM}/install/cm/include/src/cm/art/cyber)

  if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    include_directories(
            ${CMAKE_CURRENT_SOURCE_DIR}/cm/art/third_party/x86/fast-rtps/include
    )
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    include_directories(
            ${CMAKE_CURRENT_SOURCE_DIR}/cm/art/third_party/aarch64/fast-rtps/include
    )
  endif()

endif()

#file(GLOB_RECURSE CM_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/../build/${ARCH_CM}/install/lib/*.*)

add_subdirectory(cm)
add_subdirectory(proto)
add_subdirectory(common)
add_subdirectory(app)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
#安装脚本
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../scripts
        DESTINATION scripts)
