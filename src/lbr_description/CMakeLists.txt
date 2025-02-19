cmake_minimum_required(VERSION 3.22)
project(lbr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ign_ros2_control REQUIRED)

# install
install(
  DIRECTORY config meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.dsv")

if(BUILD_TESTING)
  ament_add_pytest_test(test_urdf ${CMAKE_CURRENT_SOURCE_DIR}/test/test_urdf.py)
endif()

ament_package()
