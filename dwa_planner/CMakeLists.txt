cmake_minimum_required(VERSION 3.8)
project(dwa_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(tf_transformations REQUIRED)



ament_python_install_package(${PROJECT_NAME})



# Install Python executables
install(PROGRAMS
  scripts/dwa_planner_node.py
  scripts/dwa_node.py
  scripts/new_dwa_planner.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files.
install(DIRECTORY
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
