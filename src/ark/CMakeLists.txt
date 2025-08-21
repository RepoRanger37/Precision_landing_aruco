cmake_minimum_required(VERSION 3.8)
project(ark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(cam src/aruco.cpp)
ament_target_dependencies(cam rclcpp sensor_msgs geometry_msgs std_msgs image_transport cv_bridge OpenCV)
add_executable(aruco src/aruco_detect.cpp)
ament_target_dependencies(aruco 
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(fb src/buffer.cpp)
ament_target_dependencies(fb rclcpp sensor_msgs OpenCV cv_bridge)

add_executable(video src/video.cpp)
ament_target_dependencies(video rclcpp sensor_msgs cv_bridge OpenCV)

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

# Install executables
install(TARGETS aruco
		cam
		fb
    video
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
