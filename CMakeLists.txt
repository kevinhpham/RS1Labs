cmake_minimum_required(VERSION 3.8)
project(labs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(image_processor src/image_processor.cpp)
ament_target_dependencies(image_processor rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(lab2 src/lab2.cpp)
ament_target_dependencies(lab2 rclcpp sensor_msgs)

add_executable(lab3 src/lab3.cpp)
ament_target_dependencies(lab3 rclcpp sensor_msgs tf2 geometry_msgs nav_msgs tf2_geometry_msgs)

add_executable(lab5 src/lab5.cpp)
ament_target_dependencies(lab5 rclcpp sensor_msgs cv_bridge OpenCV geometry_msgs)

add_executable(scan_localiser src/scan_localiser.cpp)
ament_target_dependencies(scan_localiser rclcpp sensor_msgs cv_bridge OpenCV geometry_msgs nav_msgs)

add_executable(rmse_calculator src/rmse_calculator.cpp)
ament_target_dependencies(rmse_calculator  rclcpp nav_msgs geometry_msgs tf2_geometry_msgs)

add_executable(cylinder_detector src/cylinder_detector.cpp)
ament_target_dependencies(cylinder_detector rclcpp sensor_msgs cv_bridge OpenCV tf2 geometry_msgs nav_msgs tf2_geometry_msgs)

install(TARGETS
  lab3
  lab2
  lab5
  scan_localiser
  rmse_calculator
  image_processor
  cylinder_detector
  DESTINATION lib/${PROJECT_NAME})
  
  install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
  )
  
  install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
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
