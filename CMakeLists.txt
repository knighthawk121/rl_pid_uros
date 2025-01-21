cmake_minimum_required(VERSION 3.8)
project(rl_pid_uros)

# Enable C++ compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
# Generate interfaces for services and actions
# Note: Files are relative to the package root directory
rosidl_generate_interfaces(${PROJECT_NAME}
  # Make sure these files exist exactly as named
  "srv/Tminusp.srv"
  "action/TunePID.action"
  "action/TunePIDmin.action"
  DEPENDENCIES action_msgs actionlib_msgs std_msgs
)

# Install Python modules
ament_python_install_package(
  rl_pid_uros_py
)

# Install Python executables
install(PROGRAMS
  scripts/rl_pid_actions.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pid_tuner
)

install(PROGRAMS
  scripts/rl_pid.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pid_tuner_service
)



install(PROGRAMS
  scripts/launch_action.py
  DESTINATION share/${PROJECT_NAME}
  RENAME rl_action
)


install(PROGRAMS
  scripts/launch_service.py
  DESTINATION share/${PROJECT_NAME}
  RENAME rl_service
)


# Install additional package files
install(DIRECTORY
  srv
  action
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()