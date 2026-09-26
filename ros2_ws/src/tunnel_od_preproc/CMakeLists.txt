cmake_minimum_required(VERSION 3.8)
project(tunnel_od_preproc)

# Приём облака на C++: разбор в каноническое облако ядра + проверка входного потока.
# Без PCL. Флаги: -O3 и -march=x86-64-v2 (не native: образ собирают на другой машине).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(TUNNEL_OD_MARCH "x86-64-v2" CACHE STRING "-march для x86_64 (пусто -- не задавать)")
add_compile_options(-Wall -Wextra -Wpedantic)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64" AND TUNNEL_OD_MARCH)
  add_compile_options(-march=${TUNNEL_OD_MARCH})
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# разбор и проверка потока без ROS 2: узел, gtest и Python (ctypes, tunnel_od_preproc/native.py)
add_library(tunnel_od_canonical SHARED src/canonical.cpp src/monitor.cpp src/c_api.cpp)
target_include_directories(tunnel_od_canonical PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(preproc_component SHARED src/preproc_node.cpp)
target_link_libraries(preproc_component tunnel_od_canonical)
ament_target_dependencies(preproc_component
  rclcpp rclcpp_components sensor_msgs std_msgs diagnostic_msgs geometry_msgs nav_msgs tf2_msgs)
rclcpp_components_register_node(preproc_component
  PLUGIN "tunnel_od::PreprocNode"
  EXECUTABLE preproc_node)

add_executable(latency_probe src/latency_probe.cpp)
ament_target_dependencies(latency_probe rclcpp sensor_msgs std_msgs)

install(TARGETS tunnel_od_canonical preproc_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS latency_probe DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_canonical test/test_canonical.cpp test/test_monitor.cpp)
  target_link_libraries(test_canonical tunnel_od_canonical)
  # копия теста в install -- чтобы ./run.sh test запускал его в готовом образе
  install(TARGETS test_canonical DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
