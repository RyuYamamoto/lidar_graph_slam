cmake_minimum_required(VERSION 3.5)
project(lidar_graph_slam_viewer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Iridescence REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${Iridescence_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ndt convergence evaluation node
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "LidarGraphSlamViewer"
  EXECUTABLE ${PROJECT_NAME}_node
)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${Iridescence_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
