#!/bin/bash

script_path=$(dirname $(readlink -f $0))
os_platform=$1
middleware=$2
cmprotocol=$3

if [ $os_platform == 'x86' ]; then
  echo "Compile CM platform library based on x86 linux!"
  toolchain_cfg=$script_path"/toolchain/toolchain_x86.cmake"
elif [ $os_platform == 'AOS' ]; then
  echo "Compile CM platform library based on AOS!"
  toolchain_cfg=$script_path"/toolchain/toolchain_aos.cmake"
elif [ $os_platform == 'BOS' ]; then
  echo "Compile CM platform library based on BST OS!"
  toolchain_cfg=$script_path"/toolchain/toolchain_bos.cmake"
elif [ $os_platform == 'ARM' ]; then
  echo "Compile CM platform library based on ARM!"
  if [ "X$(uname -m)" == Xx86_64 ];then
    toolchain_cfg="/home/cmake/toolchain.cmake"
    crosscompile=1
  else
    toolchain_cfg=$script_path"/toolchain/toolchain_arm.cmake"
  fi
elif [ $os_platform == 'J5' ]; then
  echo "Compile CM platform library based on J5!"
  toolchain_cfg=$script_path"/toolchain/toolchain_j5.cmake"
elif [ $os_platform == 'X9' ]; then
  echo "Compile CM platform library based on X9!"
  toolchain_cfg=$script_path"/toolchain/toolchain_x9.cmake"
else
  echo "Unsupported OS platform with \$1: $os_platform"
  exit
fi

if [ $middleware == 'ROS' ]; then
  echo "Compile CM platform library based on ROS!"
elif [ $middleware == 'AP' ]; then
  echo "Compile CM platform library based on AUTOSAR AP!"
  CMAKE_AP_FLAG="-DCMAKE_MW_STRATEGY=$3"
elif [ $middleware == 'RT' ]; then
  echo "Compile CM platform library based on Cyber RT!"
elif [ $middleware == 'TZ' ]; then
  echo "Compile CM platform library based on TZ!"
elif [ $middleware == 'CAP' ]; then
  echo "Compile CM platform library based on CAIC AP!"
  toolchain_cfg=$script_path"/toolchain/toolchain_cap.cmake"
elif [ $middleware == 'ROSM' ]; then
  echo "Compile CM platform library based on ROS with Generated Messages !"
elif [ $middleware == 'ART' ]; then
  echo "Compile CM platform library based on Apollo-Cyber-RT (ART, RT is bst-cyber-rt)!"
else
  echo "Unsupported middleware with \$2: $middleware"
  exit
fi

build_path=$script_path"/build/"$os_platform"/"$middleware
install_path="install"

if [ ! -d $build_path ]; then
  mkdir -p $build_path
  echo "create build folder!"
else
  #rm -rf $build_path/*
  echo "clear all temporary files!"
fi

bash ${script_path}/CI/protobuf/generate_proto.sh "${build_path}"
if [ $middleware == 'AP' ]; then
  if [ ! -z "$3" ];then
      CMAKE_AP_FLAG="-DCMAKE_MW_STRATEGY=$3"
      mkdir -p "${build_path}/$3"
  fi
  rm -rf ${script_path}/src/cm/ap/conf/ ${script_path}/src/cm/ap/generated/
  bash ${script_path}/scripts/arxml_convert_tool.sh /home/mdc/MDC_Development_Studio-Ubuntu18-2.0.011-T ${script_path}/src/cm/ap/arxml/
fi

cd $build_path

cmake  ../../.. -DCMAKE_TOOLCHAIN_FILE=$toolchain_cfg -DUSE_CROSS_COMPILE=${crosscompile} -DCMAKE_MW=$middleware ${CMAKE_AP_FLAG} -DCMAKE_BUILD_TYPE=Debug  -DCMAKE_INSTALL_PREFIX=$install_path -DCMAKE_VERSION=3.14 # ${@:3}

make -j16  # VERBOSE=1
make install
