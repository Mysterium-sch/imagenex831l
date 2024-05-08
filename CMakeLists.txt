cmake_minimum_required(VERSION 3.5)
project(imagenex831l_ros2)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RawRange.msg
  msg/ProcessedRange.msg
  DEPENDENCIES std_msgs geometry_msgs
)

## Install Python scripts
ament_python_install_package(imagenex831l)


install(DIRECTORY
  launch
  imagenex831l
  cfg
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
  imagenex831l/imagenex831l_driver.py
  imagenex831l/utilities.py
  imagenex831l/sonar_node.py
  imagenex831l/test_sonar.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
