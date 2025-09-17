cmake_minimum_required(VERSION 2.8.3)
project(global_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(/usr/include/ceres/)
file(GLOB Ceres_LIBS "/usr/lib64/libceres*.so")
file(GLOB Glog_LIBS "/usr/lib/aarch64-linux-gnu/libglog*.so")
find_package(Ceres REQUIRED)

add_subdirectory(./ThirdParty/GeographicLib/)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "/usr/include/ceres"
  "/usr/include/eigen"
  "/usr/include/glog" 
  ./ThirdParty/GeographicLib/include/
)


catkin_package()


add_executable(global_fusion_node 
	src/globalOptNode.cpp
	src/globalOpt.cpp)

target_link_libraries(global_fusion_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${Ceres_LIBRARIES} ${Ceres_LIBS} libGeographiccc ${Glog_LIBS}) 
