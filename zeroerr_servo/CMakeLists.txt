cmake_minimum_required(VERSION 3.8)
project(zeroerr_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(moveit_common REQUIRED)
# moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    trajectory_msgs)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# include_directories(include)

# Keyboard control example for servo
add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
# target_include_directories(zeroerr_servo PUBLIC include)
ament_target_dependencies(
  servo_keyboard_input 
  "control_msgs"
  "geometry_msgs"
  "moveit_core"
  "moveit_msgs"
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "pluginlib"
  "rclcpp"
  "rclcpp_components"
  "realtime_tools"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "tf2_eigen"
  "trajectory_msgs"
)

install(TARGETS servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()