cmake_minimum_required(VERSION 3.5)
project(sonar_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ezsocket
add_library(ezsocket 
  src/sonar_driver/ezsocket/ezsocket.cxx 
  src/sonar_driver/ezsocket/socketWorker.cxx
)
if (WIN32)
target_link_libraries(ezsocket wsock32 ws2_32)
endif (WIN32)

target_include_directories(ezsocket PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# sonardevices
add_library(sonardevices src/
  src/sonar_driver/sonardevices/sonardevices.cxx
)
target_link_libraries(sonardevices ezsocket)

target_include_directories(sonardevices PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

## publishers

# oculus pressure node
add_executable(oculus_pressure_node src/oculus_pressure_node.cxx)
target_link_libraries(oculus_pressure_node sonardevices)
ament_target_dependencies(oculus_pressure_node 
  rclcpp
  sensor_msgs
)
target_include_directories(oculus_pressure_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS oculus_pressure_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# oculus temperature node
add_executable(oculus_temperature_node src/oculus_temperature_node.cxx)
target_link_libraries(oculus_temperature_node sonardevices)
ament_target_dependencies(oculus_temperature_node 
  rclcpp
  sensor_msgs
)
target_include_directories(oculus_temperature_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS oculus_temperature_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# oculus image node
add_executable(oculus_image_node src/oculus_image_node.cxx)
target_link_libraries(oculus_image_node sonardevices)
ament_target_dependencies(oculus_image_node 
  rclcpp
  sensor_msgs
)
target_include_directories(oculus_image_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS oculus_image_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
