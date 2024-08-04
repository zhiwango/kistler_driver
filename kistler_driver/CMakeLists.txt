cmake_minimum_required(VERSION 3.8)
project(kistler_driver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(kistler_driver_node SHARED
  src/kistler_driver.cpp
)

rclcpp_components_register_node(kistler_driver_node
  PLUGIN "kistler_driver::KistlerDriver"
  EXECUTABLE kistler_driver)

install(PROGRAMS
  scripts/show.py
  scripts/ui_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  scripts
  launch
)
