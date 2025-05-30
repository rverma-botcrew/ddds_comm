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


# # --- Build executable ---
# add_executable(point_bridge
#   src/point_bridge.cc
#   ${GEN_DIR}/posestamped.cpp
# )

# ament_target_dependencies(point_bridge
#   rclcpp
#   geometry_msgs
# )

# target_include_directories(point_bridge PRIVATE
#   ${GEN_DIR}
#   $ENV{CXX_INSTALL}/include
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# target_link_libraries(point_bridge
#   CycloneDDS-CXX::ddscxx
#   iceoryx_binding_c::iceoryx_binding_c
# )

# # --- Install executable ---
# install(TARGETS point_bridge
#   DESTINATION lib/${PROJECT_NAME}
# )

# --- Build executable ---
# add_executable(odom_pcl_bridge
#   src/odom_pcl_bridge.cc
#   ${GEN_DIR}/posestamped.cpp
# )

# ament_target_dependencies(odom_pcl_bridge
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   message_filters
# )

# target_include_directories(odom_pcl_bridge PRIVATE
#   ${GEN_DIR}
#   $ENV{CXX_INSTALL}/include
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# target_link_libraries(odom_pcl_bridge
#   CycloneDDS-CXX::ddscxx
#   iceoryx_binding_c::iceoryx_binding_c
# )

# # --- Install executable ---
# install(TARGETS odom_pcl_bridge
#   DESTINATION lib/${PROJECT_NAME}
# )

# --- Build executable ---
add_executable(pose_pcl_bridge
  src/pose_pcl_bridge.cc
  ${GEN_DIR}/odom_pcl.cpp
)

ament_target_dependencies(pose_pcl_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  message_filters
  nav_msgs
)

target_include_directories(pose_pcl_bridge PRIVATE
  ${GEN_DIR}
  $ENV{CXX_INSTALL}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(pose_pcl_bridge
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
)
install(TARGETS pose_pcl_bridge
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(point_subscriber
  src/point_subscriber.cc
  ${GEN_DIR}/odom_pcl.cpp
)

target_include_directories(point_subscriber PRIVATE
  ${GEN_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $ENV{CXX_INSTALL}/include
)

target_link_libraries(point_subscriber
  CycloneDDS-CXX::ddscxx
  iceoryx_binding_c::iceoryx_binding_c
)

install(TARGETS point_subscriber
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
