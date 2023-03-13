cmake_minimum_required(VERSION 3.5)
project(points_prefiltering)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

find_package(PCL REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include ${PCL_INCLUDE_DIRS})

ament_auto_add_library(points_prefiltering SHARED
  src/points_prefiltering_node.cpp
)

rclcpp_components_register_node(points_prefiltering
  PLUGIN "PointsPreFiltering"
  EXECUTABLE points_prefiltering_node
)

target_link_libraries(points_prefiltering ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

install(FILES DESTINATION share/${PROJECT_NAME})
