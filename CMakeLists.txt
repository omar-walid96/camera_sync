cmake_minimum_required(VERSION 3.8)
project(cam_sync_bench)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cam_sync_msgs REQUIRED)

add_executable(sync_bench_node src/sync_bench_node.cpp)
ament_target_dependencies(sync_bench_node rclcpp sensor_msgs message_filters cam_sync_msgs)

add_executable(cam_viewer_node src/cam_viewer_node.cpp)
ament_target_dependencies(cam_viewer_node rclcpp sensor_msgs message_filters cv_bridge)
target_link_libraries(cam_viewer_node ${OpenCV_LIBS})

add_executable(synced_viewer_node src/synced_viewer_node.cpp)
ament_target_dependencies(synced_viewer_node rclcpp sensor_msgs message_filters cv_bridge cam_sync_msgs)
target_link_libraries(synced_viewer_node ${OpenCV_LIBS})




install(TARGETS sync_bench_node cam_viewer_node synced_viewer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Python nodes/scripts — installed to lib so ros2 run can find them.
install(PROGRAMS
  scripts/opencv_cam_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME opencv_cam_node
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
