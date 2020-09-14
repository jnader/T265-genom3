project(T265_calib)

cmake_minimum_required(VERSION 2.6)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(ViconDataStreamSDK REQUIRED)
find_package(VISP REQUIRED)

if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif()

set(SRC_COMMON SDK_wrapper.cpp SDK_wrapper.h ViconSDK.h)

add_executable(mk-acquire-calib-data mk-acquire-calib-data.cpp ${SRC_COMMON})
target_link_libraries(mk-acquire-calib-data ${ViconDataStreamSDK_LIBRARIES})
