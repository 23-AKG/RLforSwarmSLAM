cmake_minimum_required(VERSION 3.8)
project(swarm_slam_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  swarm_slam_sim/explorer.py
  swarm_slam_sim/pointcloud_relay.py
  swarm_slam_sim/odom_to_path.py
  swarm_slam_sim/waypoint_navigator.py
  swarm_slam_sim/map_recorder.py
  swarm_slam_sim/broker_selector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  urdf
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install visualizer script to share dir
install(FILES
  visualize_maps.py
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  swarm_slam_sim/unified_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
