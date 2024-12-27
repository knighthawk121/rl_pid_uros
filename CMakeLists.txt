cmake_minimum_required(VERSION 3.8)
project(rl_pid_uros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PIDValues.msg"
  "srv/Tminusp.srv"
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the Python package
ament_python_install_package(rl_pid_uros_py)
# Install Python scripts
install(PROGRAMS
  scripts/rl_pid.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files if needed (e.g., message, service files)
install(DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
