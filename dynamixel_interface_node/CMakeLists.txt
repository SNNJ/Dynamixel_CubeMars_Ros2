cmake_minimum_required(VERSION 3.8)
project(dynamixel_interface_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
# find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(dynamixel_interface_node src/dynamixel_interface_node.cpp)
target_include_directories(dynamixel_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamixel_interface_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dynamixel_interface_node
  "rclcpp"
  "dynamixel_sdk"
#  "dynamixel_sdk_custom_interfaces"
  "rcutils"
  "std_msgs"
)

install(TARGETS dynamixel_interface_node
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
