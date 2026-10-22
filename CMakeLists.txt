cmake_minimum_required(VERSION 3.16)
project(esp32_motor_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIS_PACKAGE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/esp32_interface.cpp
  src/esp32_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface esp32_motor_hardware_interface.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
