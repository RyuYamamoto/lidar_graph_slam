cmake_minimum_required(VERSION 3.5)
project(lidar_scan_matcher)

#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 17)
#  set(CMAKE_CXX_STANDARD_REQUIRED ON)
#  set(CMAKE_CXX_EXTENSIONS OFF)
#endif()
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic )
#endif()

find_package(PCL REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/kiss-icp/cpp/kiss_icp ${CMAKE_CURRENT_BINARY_DIR}/kiss_icp)

include_directories(include ${PCL_INCLUDE_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "LidarScanMatcher"
  EXECUTABLE ${PROJECT_NAME}_node
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})


ament_auto_add_library(kiss_icp_matcher SHARED
  src/kiss_icp_matcher.cpp
)

rclcpp_components_register_node(kiss_icp_matcher
  PLUGIN "KissIcpMatcher"
  EXECUTABLE kiss_icp_matcher_node
)

target_compile_features(kiss_icp_matcher PUBLIC cxx_std_20)
target_include_directories(kiss_icp_matcher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kiss_icp_matcher ${PCL_LIBRARIES} kiss_icp::pipeline)

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
