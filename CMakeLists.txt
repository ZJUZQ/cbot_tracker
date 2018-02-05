cmake_minimum_required(VERSION 2.8.3)
project(ros_visual_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## System dependencies are found with CMake's conventions
find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)
find_package( OpenCV REQUIRED )


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  $ENV{HOME}/3rdparty/include
  $ENV{HOME}/3rdparty/ipc/include
  $ENV{HOME}/3rdparty/ipc/include/hc_include
  ${Boost_INCLUDE_DIRS}
)

link_directories( 
  $ENV{HOME}/3rdparty/lib 
  $ENV{HOME}/3rdparty/ipc/lib 
  $ENV{HOME}/3rdparty/ipc/lib/hc_lib
  ${Boost_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(cbot
  src/common_utility.cpp
)
target_link_libraries(cbot
  ${OpenCV_LIBS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(visual_localization src/visual_localization.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(visual_localization
  ${catkin_LIBRARIES}
	${OpenCV_LIBS}
	$ENV{HOME}/3rdparty/lib/libBOOSTING.so
  $ENV{HOME}/3rdparty/lib/libsakUtils.a
  $ENV{HOME}/3rdparty/ipc/lib/libIPC_CAMERA.so 

  cbot
  ${Boost_LIBRARIES}
)
