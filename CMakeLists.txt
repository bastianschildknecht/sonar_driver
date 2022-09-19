cmake_minimum_required(VERSION 3.5)
project(sonar_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sonar_driver_interfaces REQUIRED)

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

# oculusDriver
add_library(oculusDriver src/
  src/sonar_driver/oculusDriver/oculusDriverNode.cxx
)
ament_target_dependencies(oculusDriver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  sonar_driver_interfaces)

target_include_directories(oculusDriver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# oculus driver node
add_executable(oculus_driver src/oculus_driver.cxx)
target_link_libraries(oculus_driver sonardevices oculusDriver)
ament_target_dependencies(oculus_driver 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)
target_include_directories(oculus_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS oculus_driver
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
