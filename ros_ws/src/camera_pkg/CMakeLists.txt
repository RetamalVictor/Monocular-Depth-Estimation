cmake_minimum_required(VERSION 3.5)
project(camera_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)

include_directories(include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(camera_component SHARED
  src/camera_component.cpp)

target_compile_definitions(camera_component
  PRIVATE "CAMERA_PKG_BUILDING_DLL")

target_link_libraries(camera_component
  ${sensor_msgs_TARGETS}
  ${OpenCV_LIBS})
ament_target_dependencies(camera_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  )
rclcpp_components_register_nodes(camera_component "camera_composition::CameraComponent")
set(node_plugins "${node_plugins}camera_composition::CameraComponent;$<TARGET_FILE:camera_component>\n")

add_library(display_component SHARED
  src/display_component.cpp)

target_compile_definitions(display_component
  PRIVATE "CAMERA_PKG_BUILDING_DLL")

target_link_libraries(display_component
  ${sensor_msgs_TARGETS}
  ${OpenCV_LIBS})
  
ament_target_dependencies(display_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  )
rclcpp_components_register_nodes(display_component "camera_composition::DisplayComponent")
set(node_plugins "${node_plugins}camera_composition::DisplayComponent;$<TARGET_FILE:display_component>\n")

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

install(TARGETS
  camera_component
  display_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
