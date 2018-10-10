cmake_minimum_required(VERSION 2.8)
project(cc_test)

#set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
add_definitions(-std=c++11 -O2 -g -DNDEBUG)


find_package(Boost REQUIRED)
find_package(VISP REQUIRED)
find_package( OpenCV REQUIRED )
# 官方没有提供.cmake文件，https://github.com/thewtex/jsoncpp-config-cmake 仓库提供了一个.cmake文件
#find_package(jsoncpp REQUIRED)

include_directories(
  include
${Boost_INCLUDE_DIR}
"/usr/include/eigen3" 
${OpenCV_INCLUDE_DIRS}
${VISP_INCLUDE_DIRS}
)

add_executable(cc_test src/cc_test.cpp)
target_link_libraries(cc_test
${Boost_LIBRARIES}
${OpenCV_LIBS}
${VISP_LIBRARIES}
jsoncpp
)

