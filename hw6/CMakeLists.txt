cmake_minimum_required(VERSION 3.8)
project(hw6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

#Packages needed for this assignment
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#need our custom msg/srvs:
find_package(hw_interfaces REQUIRED)

#executables in this package!
add_executable(oscope src/oscope.cpp)
ament_target_dependencies(oscope rclcpp std_msgs)

add_executable(limiter src/limiter.cpp)
ament_target_dependencies(limiter rclcpp std_msgs)

add_executable(data_sender src/data_sender.cpp)
ament_target_dependencies(data_sender rclcpp hw_interfaces)

add_executable(data_receiver src/data_receiver.cpp)
ament_target_dependencies(data_receiver rclcpp hw_interfaces std_msgs)

#install the nodes so we can run them:
install(TARGETS
	oscope
	limiter
	data_sender
	data_receiver

	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	config
	DESTINATION share/${PROJECT_NAME}
)

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
