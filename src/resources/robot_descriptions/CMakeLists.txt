cmake_minimum_required(VERSION 3.8)
project(robot_descriptions)

if($ENV{ROS_VERSION} EQUAL 1)
  # In ROS1 compilation environment
  message("Currently in ROS1 compilation environment")

  find_package(catkin REQUIRED COMPONENTS roscpp)
  
  catkin_package(CATKIN_DEPENDS)

  # Install directories 'meshes' and 'urdf' to the package share directory
  install(DIRECTORY pointfoot wheellegged DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
elseif($ENV{ROS_VERSION} EQUAL 2)
  # In ROS2 compilation environment
  message("Currently in ROS2 compilation environment")

  find_package(ament_cmake REQUIRED)

  install(DIRECTORY wheelbipe25_v3 wheelbipe25_v3_serial DESTINATION share/${PROJECT_NAME})

  ament_package()
endif()