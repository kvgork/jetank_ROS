cmake_minimum_required(VERSION 3.0.2)
project(robot_main)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  pcl_ros # For pcl_conversions
  message_filters
)

## Find OpenCV and PCL for image processing and point cloud handling
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc calib3d)
find_package(PCL REQUIRED COMPONENTS common io)

## Find CUDA for GPU acceleration
find_package(CUDA REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES robot_main
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs image_transport pcl_ros message_filters
  # DEPENDS libopencv-dev libpcl-dev
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # Explicitly add the OpenCV 4 include directory to ensure correct headers are used
  /usr/local/include/opencv4 
  ${OpenCV_INCLUDE_DIRS} # Keep this for other OpenCV include paths
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS} # Include CUDA headers
)

## Declare a C++ executable for stereo processor node
add_executable(stereo_processor src/robot_main/camera/stereo_processor_c.cpp) # Ensure this path is correct

## Add cmake target dependencies of the executable
add_dependencies(stereo_processor ${catkin_EXPORTED_TARGETS})

## Specify libraries to link your executable target against
target_link_libraries(stereo_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  # Explicitly link OpenCV CUDA modules that are used
  opencv_cudaarithm
  opencv_cudaimgproc
  opencv_cudastereo
  # Add other CUDA modules if you enabled them in CMake build, e.g.:
  # opencv_cudafilters
  # opencv_cudawarping
  # opencv_cudafeatures2d
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
install(TARGETS stereo_processor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation (if you create any custom libraries)
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_main.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
