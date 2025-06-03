cmake_minimum_required(VERSION 3.5)
project(dds_comm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# --- ROS 2 dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

list (APPEND CMAKE_PREFIX_PATH
  "$ENV{CYCLONE_INSTALL}"
  "$ENV{CXX_INSTALL}"
  )

# --- Cyclone DDS C++ bindings ---
find_package(CycloneDDS-CXX CONFIG REQUIRED)
find_package(iceoryx_binding_c CONFIG REQUIRED)

set(GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")

# --- Build executable ---
add_executable(pcl_odom_bridge
  src/pcl_odom_bridge.cc
  ${GEN_DIR}/odom_pcl.cpp
)

ament_target_dependencies(pcl_odom_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  message_filters
  nav_msgs
)

target_include_directories(pcl_odom_bridge PRIVATE
  ${GEN_DIR}
  $ENV{CXX_INSTALL}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(pcl_odom_bridge
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
)
install(TARGETS pcl_odom_bridge
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(pcl_odom_subscriber
  src/pcl_odom_subscriber.cc
  ${GEN_DIR}/odom_pcl.cpp
)

target_include_directories(pcl_odom_subscriber PRIVATE
  ${GEN_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $ENV{CXX_INSTALL}/include
)

target_link_libraries(pcl_odom_subscriber
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
)

install(TARGETS pcl_odom_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(image_odom_bridge
  src/image_odom_bridge.cc
  ${GEN_DIR}/odom_pcl.cpp
)

ament_target_dependencies(image_odom_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  message_filters
  nav_msgs
)

target_include_directories(image_odom_bridge PRIVATE
  ${GEN_DIR}
  $ENV{CXX_INSTALL}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(image_odom_bridge
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
)

install(TARGETS image_odom_bridge
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(image_odom_subscriber
  src/image_odom_subscriber.cc
  ${GEN_DIR}/odom_pcl.cpp
)

target_include_directories(image_odom_subscriber PRIVATE
  ${GEN_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $ENV{CXX_INSTALL}/include
)

target_link_libraries(image_odom_subscriber
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
)

install(TARGETS image_odom_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(dummy_odom_publisher
  src/dummy_odom_publisher.cpp
  # ${GEN_DIR}/odom_pcl.cpp
)

ament_target_dependencies(dummy_odom_publisher
  rclcpp
  geometry_msgs
  sensor_msgs
  message_filters
  nav_msgs
)

install(TARGETS dummy_odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
