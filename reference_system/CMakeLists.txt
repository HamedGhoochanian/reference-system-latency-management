cmake_minimum_required(VERSION 3.5)
project(reference_system
  VERSION 0.0.1
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add header-only library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(${BUILD_TESTING})
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sample_management test/test_sample_management.cpp)
  ament_target_dependencies(test_sample_management rclcpp reference_interfaces)
  target_link_libraries(test_sample_management ${PROJECT_NAME})
  ament_add_gtest(test_input_consumption test/test_input_consumption.cpp)
  ament_target_dependencies(test_input_consumption rclcpp reference_interfaces)
  target_link_libraries(test_input_consumption ${PROJECT_NAME})
  add_executable(test_source_schedule test/test_source_schedule.cpp)
  target_link_libraries(test_source_schedule ${PROJECT_NAME})
  add_test(NAME test_source_schedule COMMAND test_source_schedule)
  add_executable(test_telemetry_buffer test/telemetry_buffer_harness.cpp)
  target_link_libraries(test_telemetry_buffer ${PROJECT_NAME})
  add_test(NAME test_telemetry_buffer COMMAND test_telemetry_buffer)
  add_executable(test_tracking test/tracking_harness.cpp)
  target_link_libraries(test_tracking ${PROJECT_NAME})
  add_test(NAME test_tracking COMMAND test_tracking)
endif()

# Install
install(TARGETS ${PROJECT_NAME}
        EXPORT "export_${PROJECT_NAME}"
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION include
)

ament_export_targets("export_${PROJECT_NAME}")
ament_auto_package()
