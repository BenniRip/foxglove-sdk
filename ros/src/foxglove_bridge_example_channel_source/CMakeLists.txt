cmake_minimum_required(VERSION 3.17)
project(foxglove_bridge_example_channel_source LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(foxglove_bridge REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/example_channel_source.cpp)
target_link_libraries(${PROJECT_NAME}
  foxglove_bridge::plugin_api
  pluginlib::pluginlib
  rclcpp::rclcpp
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)

pluginlib_export_plugin_description_file(foxglove_bridge plugins.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(plugin_loading_test test/plugin_loading_test.cpp)
  target_link_libraries(plugin_loading_test
    foxglove_bridge::plugin_api
    pluginlib::pluginlib
    rclcpp::rclcpp
  )
endif()

ament_package()
