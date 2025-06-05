cmake_minimum_required(VERSION 3.5)
project(ros2in1_support)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_msgs
)
include(cmake/ros2in1_support-extras.cmake)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_msgs
  CFG_EXTRAS ros2in1_support-extras.cmake
)
string(REPLACE ";" ":" AMENT_PREFIX_PATH "${ROS2_CMAKE_PREFIX_PATH}")
catkin_add_env_hooks(ros2in1_support-env SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

#######################################################################
## Find and install general and DDS-vendor specific ROS 2 libraries  ##
## to the ROS 1 install-space (and symlink them to the devel-space). ##
#######################################################################

if(RMW_IMPLEMENTATION STREQUAL "rmw_fastrtps_cpp")
  if(ROS2_DISTRO STREQUAL "galactic")
    set(_rmw_library_names
      fastcdr
      fastrtps
      rcpputils
      rcutils
      rmw
      rmw_dds_common
      rmw_dds_common__rosidl_typesupport_cpp
      rmw_dds_common__rosidl_typesupport_introspection_cpp
      rmw_fastrtps_cpp
      rmw_fastrtps_shared_cpp
      rosidl_runtime_c
      rosidl_typesupport_cpp
      rosidl_typesupport_fastrtps_c
      rosidl_typesupport_fastrtps_cpp
    )
  else()
    set(_rmw_library_names
      fastcdr
      fastrtps
      rcpputils
      rcutils
      rmw
      rmw_dds_common
      rmw_dds_common__rosidl_typesupport_cpp
      rmw_dds_common__rosidl_typesupport_fastrtps_cpp
      rmw_dds_common__rosidl_typesupport_introspection_cpp
      rmw_fastrtps_cpp
      rmw_fastrtps_shared_cpp
      rosidl_runtime_c
      rosidl_typesupport_cpp
      rosidl_typesupport_fastrtps_c
      rosidl_typesupport_fastrtps_cpp
      rosidl_typesupport_introspection_c
      rosidl_typesupport_introspection_cpp
      tracetools
    )
  endif()
elseif(RMW_IMPLEMENTATION STREQUAL "rmw_cyclonedds_cpp")
  if(ROS2_DISTRO STREQUAL "galactic")
    set(_rmw_library_names
      ddsc
      iceoryx_binding_c
      iceoryx_platform
      iceoryx_posh
      iceoryx_utils
      rcpputils
      rcutils
      rmw
      rmw_cyclonedds_cpp
      rmw_dds_common
      rmw_dds_common__rosidl_typesupport_cpp
      rmw_dds_common__rosidl_typesupport_introspection_cpp
      rosidl_runtime_c
      rosidl_typesupport_cpp
      rosidl_typesupport_introspection_c
      rosidl_typesupport_introspection_cpp
    )
  else()
    set(_rmw_library_names
      ddsc
      iceoryx_binding_c
      iceoryx_hoofs
      iceoryx_platform
      iceoryx_posh
      rcpputils
      rcutils
      rmw
      rmw_cyclonedds_cpp
      rmw_dds_common
      rmw_dds_common__rosidl_typesupport_cpp
      rmw_dds_common__rosidl_typesupport_introspection_cpp
      rosidl_runtime_c
      rosidl_typesupport_cpp
      rosidl_typesupport_introspection_c
      rosidl_typesupport_introspection_cpp
      tracetools
    )
  endif()
endif()
if(NOT _rmw_library_names)
  message(FATAL_ERROR "Selected RMW implemenation '${RMW_IMPLEMENTATION}' for ROS ${ROS2_DISTRO} is currently not supported by ${PROJECT_NAME}.")
endif()
foreach(_library ${_rmw_library_names})
  find_ros2_library(${_library})
endforeach()

###########
## Build ##
###########

# ROS 2 support may not actually be enabled (-DROS2_SUPPORT=ON).
# We still build this library, which then provides ROS 1 only
# wrappers for publishers, service servers and tf.
if(ROS2_SUPPORT)
  add_definitions(-DROS2_SUPPORT)
  find_ros2_package(rclcpp REQUIRED)
  find_ros2_package(tf2_msgs REQUIRED)
  set(ros2in1_support_TARGETS ${rclcpp_TARGETS_ROS2} ${tf2_msgs_TARGETS_ROS2})
  use_ros2_libraries(${ros2in1_support_TARGETS})
else()
  message(WARNING "Building ros2in1_support without ROS 2 support!")
endif()

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
add_library(
  ${PROJECT_NAME}
  src/node.cpp
  src/tf.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${ros2in1_support_TARGETS}
)

#############
## Install ##
#############

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
