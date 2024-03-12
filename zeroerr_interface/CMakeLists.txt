cmake_minimum_required(VERSION 3.8)
project(zeroerr_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

ament_export_include_directories(
  include
  ${ETHERLAB_DIR}/include
)

# add_library(
#   ${PROJECT_NAME}
#   SHARED
#   src/zeroerr_interface.cpp)

add_executable(zeroerr_interface src/zeroerr_interface.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${ETHERLAB_DIR}/include
)

target_link_libraries(${PROJECT_NAME} ${ETHERCAT_LIB})

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
  ${ETHERCAT_LIBRARY}
)
ament_export_dependencies(
  rclcpp
  sensor_msgs
)
ament_package()
