cmake_minimum_required(VERSION 3.5)
project(my_multi_service_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(multi_service_node src/multi_service_node.cpp)
ament_target_dependencies(multi_service_node rclcpp example_interfaces std_srvs)

install(TARGETS
  multi_service_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
