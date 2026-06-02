cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install python scripts
install(PROGRAMS
  src/speed.py
  src/colorize_cloud.py
  src/depth_colorize_cloud.py
  src/imu_republisher.py
  src/odom_kalman.py
  src/scan_to_cloud.py
  src/kalman.py
  src/odom_noisy.py
  src/fusion_node.py
  src/ekf_cloud.py
  src/segment_cloud.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install URDF/Xacro
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz config
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()


# Optional resources
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/worlds)
  install(
    DIRECTORY worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  install(
    DIRECTORY include/
    DESTINATION include
  )
endif()
