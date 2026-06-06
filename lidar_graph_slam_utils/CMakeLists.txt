cmake_minimum_required(VERSION 3.14)
project(lidar_graph_slam_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# GTSAM exports an imported target (gtsam) and GTSAM_INCLUDE_DIR rather than the
# ament-style gtsam_INCLUDE_DIRS/gtsam_LIBRARIES variables, so ament_cmake_auto
# does not propagate its include paths automatically. Link the target explicitly.
find_package(GTSAM REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)

target_link_libraries(${PROJECT_NAME} gtsam)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
