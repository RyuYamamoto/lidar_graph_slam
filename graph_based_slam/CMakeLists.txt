cmake_minimum_required(VERSION 3.5)
project(graph_based_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})

ament_auto_add_library(graph_based_slam SHARED
  src/graph_based_slam.cpp
)

rclcpp_components_register_node(graph_based_slam
  PLUGIN "GraphBasedSLAM"
  EXECUTABLE graph_based_slam_node
)

target_link_libraries(graph_based_slam ${PCL_LIBRARIES} ${GTSAM_LIBRARY_DIRS} gtsam)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

install(FILES DESTINATION share/${PROJECT_NAME})
