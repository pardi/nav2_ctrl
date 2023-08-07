find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(test_dex_controller
  test_dex_controller.cpp
)
ament_target_dependencies(test_dex_controller
  ${dependencies}
)
target_link_libraries(test_dex_controller
  ${library_name}
)