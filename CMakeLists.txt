cmake_minimum_required(VERSION 3.8)
project(px4adrc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlatTrajectoryReference.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(px4adrc_blocks src/adrc_blocks.cpp)
target_link_libraries(px4adrc_blocks Eigen3::Eigen)
target_include_directories(px4adrc_blocks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(px4adrc_core src/adrc_controller.cpp)
target_link_libraries(px4adrc_core Eigen3::Eigen px4adrc_blocks)
target_include_directories(px4adrc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(px4adrc_node_lib src/adrc_node.cpp)
target_link_libraries(px4adrc_node_lib px4adrc_core "${cpp_typesupport_target}")
ament_target_dependencies(px4adrc_node_lib rclcpp px4_msgs std_msgs geometry_msgs)
target_include_directories(px4adrc_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(px4adrc_node src/adrc_node_main.cpp)
target_link_libraries(px4adrc_node px4adrc_node_lib)
ament_target_dependencies(px4adrc_node rclcpp)
target_include_directories(px4adrc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  ament_add_gtest(test_common test/test_common.cpp)
  target_link_libraries(test_common "${cpp_typesupport_target}")
  target_link_libraries(test_common Eigen3::Eigen)
  target_include_directories(test_common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(test_adrc_blocks test/test_adrc_blocks.cpp)
  target_link_libraries(test_adrc_blocks px4adrc_blocks)
  target_include_directories(test_adrc_blocks PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(test_adrc_controller test/test_adrc_controller.cpp)
  target_link_libraries(test_adrc_controller px4adrc_core)
  target_include_directories(test_adrc_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(test_adrc_node_logic test/test_adrc_node_logic.cpp)
  target_link_libraries(test_adrc_node_logic px4adrc_node_lib)
  target_include_directories(test_adrc_node_logic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

install(TARGETS px4adrc_blocks px4adrc_core px4adrc_node_lib px4adrc_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/flatness_reference_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
