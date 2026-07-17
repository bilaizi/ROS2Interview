# my_multi_service_pkg

This package provides a simple ROS 2 C++ node that offers multiple services from a single node.

Files added:
- src/multi_service_node.cpp
- CMakeLists.txt
- package.xml

Branch: add/multi-service-node

Build

1. From your ROS 2 workspace root (where this repository is checked out), run:

   colcon build --packages-select my_multi_service_pkg

2. Source the install setup:

   source install/setup.bash

Run

- Start the node:

  ros2 run my_multi_service_pkg multi_service_node

- Call the services from another terminal:

  ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"
  ros2 service call /set_flag std_srvs/srv/SetBool "{data: true}"

Notes

- The node uses a MultiThreadedExecutor so both services can be handled concurrently.
- If you'd like a different package name or placement, let me know and I can update it.
