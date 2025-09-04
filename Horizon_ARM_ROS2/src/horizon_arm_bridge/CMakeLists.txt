cmake_minimum_required(VERSION 3.5)
project(horizon_arm_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(FILES
  scripts/trajectory_stream_sdk_driver.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)

# Install algorithm module as a regular file (not executable)
install(FILES
  scripts/sync_movement_algorithm.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


