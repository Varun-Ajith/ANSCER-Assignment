cmake_minimum_required(VERSION 3.8)
project(trajectory_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)

# Generate the custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Include directories
include_directories(include)

# Get the typesupport target for the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Trajectory Publisher and Saver Node
add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
ament_target_dependencies(trajectory_publisher_saver
  rclcpp 
  std_msgs 
  geometry_msgs 
  visualization_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)
target_link_libraries(trajectory_publisher_saver "${cpp_typesupport_target}")

# Trajectory Reader and Publisher Node
add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)
ament_target_dependencies(trajectory_reader_publisher
  rclcpp 
  std_msgs 
  geometry_msgs 
  visualization_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  nlohmann_json
)
target_link_libraries(trajectory_reader_publisher "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  trajectory_publisher_saver
  trajectory_reader_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()