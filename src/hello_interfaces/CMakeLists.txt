cmake_minimum_required(VERSION 3.8)
project(hello_interfaces)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Uncomment the following section if you need to add further dependencies manually.
# find_package(<dependency> REQUIRED)

# List of message files
set(msg_files
  "msg/MyString.msg"
)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

