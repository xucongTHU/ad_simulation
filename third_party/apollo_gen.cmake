

function(PROTOBUF_GENERATE_SOURCE SRCS HDRS PYS PROTOBUF_IMPORT_DIRS PROTOBUF_CPP_EXPORT_DIR PROTOBUF_PYTHON_EXPORT_DIR)
  if(NOT ARGN)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE_SOURCE() called without any proto files")
    return()
  endif()

  if(DEFINED PROTOBUF_IMPORT_DIRS)
    foreach(DIR ${PROTOBUF_IMPORT_DIRS})
      get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  endif()

  set(${SRCS})
  set(${HDRS})
  set(${PYS})
  foreach(FIL ${ARGN})
    set(ABS_FIL ${PROTOBUF_IMPORT_DIRS}/${FIL})
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    get_filename_component(RELATIVE_PATH ${FIL} PATH)

    list(APPEND ${SRCS} "${PROTOBUF_CPP_EXPORT_DIR}/${RELATIVE_PATH}/${FIL_WE}.pb.cc")
    list(APPEND ${HDRS} "${PROTOBUF_CPP_EXPORT_DIR}/${RELATIVE_PATH}/${FIL_WE}.pb.h")
    list(APPEND ${PYS} "${PROTOBUF_PYTHON_EXPORT_DIR}/${RELATIVE_PATH}/${FIL_WE}_pb2.py")

    add_custom_command(
      OUTPUT "${PROTOBUF_CPP_EXPORT_DIR}/${RELATIVE_PATH}/${FIL_WE}.pb.cc"
             "${PROTOBUF_CPP_EXPORT_DIR}/${RELATIVE_PATH}/${FIL_WE}.pb.h"
             "${PROTOBUF_PYTHON_EXPORT_DIR}/${RELATIVE_PATH}/${FIL_WE}_pb2.py"
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out ${PROTOBUF_CPP_EXPORT_DIR}
           --python_out ${PROTOBUF_PYTHON_EXPORT_DIR}
           -I${PROTOBUF_IMPORT_DIRS} ${ABS_FIL}
      DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Running C++/Python protocol buffer compiler on ${FIL}"
      VERBATIM)
  endforeach()

  set_source_files_properties(${${SRCS}} ${${HDRS}} ${${PYS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
  set(${PYS} ${${PYS}} PARENT_SCOPE)
endfunction()

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/..)
include_directories(${PROJECT_INCLUDE_DIRS}/..)

find_package(Boost REQUIRED COMPONENTS iostreams)
include_directories(${Boost_INCLUDE_DIRS})



set(PROTOBUF_IMPORT_DIRS ../src)
set(PROTOBUF_CPP_EXPORT_DIR ${CMAKE_CURRENT_BINARY_DIR}/..)
set(PROTOBUF_PYTHON_EXPORT_DIR ${CMAKE_CURRENT_BINARY_DIR}/..)

FILE(GLOB_RECURSE ProtoFiles RELATIVE ../src *.proto)

PROTOBUF_GENERATE_SOURCE(
  PROTOSRCS PROTOHDRS PYS
  ${PROTOBUF_IMPORT_DIRS}
  ${PROTOBUF_CPP_EXPORT_DIR}
  ${PROTOBUF_PYTHON_EXPORT_DIR}
  ${ProtoFiles}
)

add_library(smartsim SHARED ${PROTOSRCS}
        smartsim/hdmap/hdmap_common.cc
        smartsim/hdmap/hdmap.cc
        smartsim/hdmap/hdmap_impl.cc
        smartsim/hdmap/hdmap_util.cc
        smartsim/proto_utils.cc
        smartsim/math/aabox2d.cc
        smartsim/math/angle.cc
        smartsim/math/box2d.cc
        smartsim/math/cartesian_frenet_conversion.cc
        smartsim/math/linear_interpolation.cc
        smartsim/math/line_segment2d.cc
        smartsim/math/math_utils.cc
        smartsim/math/polygon2d.cc
        smartsim/math/qp_solver.cc
        smartsim/math/vec2d.cc
        smartsim/math/search.cc
        smartsim/math/sin_table.cc
        smartsim/math/linear_quadratic_regulator.cc
        smartsim/pnc_map/path.cc
        smartsim/pnc_map/route_segments.cc
        smartsim/pnc_map/pnc_map.cc
        smartsim/common/file.cc
        smartsim/common/vehicle_config_helper.cc
        smartsim/common/config_gflags.cc
)

find_package(gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

target_link_libraries(smartsim
                      ${Boost_LIBRARIES}
                      ${PROTOBUF_LIBRARIES}
                      ${GFLAGS_LIBRARIES}
)

install(TARGETS smartsim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/proto
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.pb.h"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/smartsim
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.pb.h"
)


