# Copyright (c) 2024, Intermodalics BV
# All rights reserved.
# CMake macros and functions to help finding and using ROS 2 dependencies in ROS 1 packages.

cmake_minimum_required(VERSION 3.15)

if(ros2in1_support_SOURCE_PREFIX)
  set(ros2in1_support_PACKAGE_DIR ${ros2in1_support_SOURCE_PREFIX})
else()
  set(ros2in1_support_PACKAGE_DIR ${ros2in1_support_DIR}/..)
endif()

# *****************************************************************************
#  Definitions
# *****************************************************************************

# Set ROS2_DISTRO and RMW_IMPLEMENTATION from environment.
if(NOT DEFINED ROS2_DISTRO)
  set(ROS2_DISTRO "$ENV{ROS2_DISTRO}")
endif()
if(NOT DEFINED RMW_IMPLEMENTATION)
  if(NOT "$ENV{RMW_IMPLEMENTATION}" STREQUAL "")
    set(RMW_IMPLEMENTATION "$ENV{RMW_IMPLEMENTATION}")
  else()
    set(RMW_IMPLEMENTATION "rmw_fastrtps_cpp")
    #set(RMW_IMPLEMENTATION "rmw_cyclonedds_cpp")
  endif()
endif()

# Find the CMAKE_PREFIX_PATH and PYTHONPATH to be used for ROS 2 packages (e.g. /opt/ros/humble).
if(NOT ROS2_CMAKE_PREFIX_PATH)
  if(ROS2_DISTRO AND EXISTS "/opt/ros/${ROS2_DISTRO}")
    set(ROS2_CMAKE_PREFIX_PATH "/opt/ros/${ROS2_DISTRO}")
    if(EXISTS "$ENV{WORKSPACE}/ros2_workspace")
      set(ROS2_CMAKE_PREFIX_PATH "$ENV{WORKSPACE}/ros2_workspace/install" ${ROS2_CMAKE_PREFIX_PATH})
    endif()
    message(STATUS "[ros2in1_support] Using the ROS 2 distribution found at ${ROS2_CMAKE_PREFIX_PATH}.")
  else()

    message(WARNING
        "[ros2in1_support] Package ${PROJECT_NAME} directly or indirectly depends on ros2in1_support, but neither "
        "the ROS2_DISTRO not the ROS2_CMAKE_PREFIX_PATH CMake or environment variable "
        "is set. Skipping ROS 2 support..."
    )
    unset(ROS2_DISTRO)
    return()
  endif()
endif()
if(NOT ROS2_PYTHONPATH)
  # TODO: Find the PYTHONPATH by sourcing the ROS 2 workspace in a child process and checking PYTHONPATH.
  find_package(Python3 REQUIRED)
  set(ROS2_PYTHONPATH)
  foreach(_path ${ROS2_CMAKE_PREFIX_PATH})
    list(APPEND ROS2_PYTHONPATH
      "${_path}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
      "${_path}/local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages"
    )
  endforeach()
endif()
string(REPLACE ";" ":" ROS2_PYTHONPATH "${ROS2_PYTHONPATH}")

message(STATUS "[ros2in1_support] ROS2_DISTRO = ${ROS2_DISTRO}")
message(STATUS "[ros2in1_support] ROS2_CMAKE_PREFIX_PATH = ${ROS2_CMAKE_PREFIX_PATH}")
message(STATUS "[ros2in1_support] ROS2_PYTHONPATH = ${ROS2_PYTHONPATH}")
message(STATUS "[ros2in1_support] RMW_IMPLEMENTATION = ${RMW_IMPLEMENTATION}")

# Find catkin, for CATKIN_DEVEL_PREFIX.
if(NOT CATKIN_DEVEL_PREFIX)
  find_package(catkin QUIET)
endif()

# *****************************************************************************
#  Helper functions
# *****************************************************************************

# Helper function to make a backup of the CMake Cache.
# That is to be able to restore the backup after each function call that may modify the cache
# (and pollute it with the ROS 2 paths of packages that have the same name in ROS 1).
function(_backup_cmake_cache _output_file)
  file(WRITE "${_output_file}" "")
  get_cmake_property(_cache_variables CACHE_VARIABLES)
  foreach(_var ${_cache_variables})
    get_property(_advanced CACHE "${_var}" PROPERTY ADVANCED)
    get_property(_helpstring CACHE "${_var}" PROPERTY HELPSTRING)
    get_property(_strings CACHE "${_var}" PROPERTY STRINGS)
    get_property(_type CACHE "${_var}" PROPERTY TYPE)
    get_property(_value CACHE "${_var}" PROPERTY VALUE)
    file(APPEND "${_output_file}" "set(${_var} \"${_value}\" CACHE ${_type} \"${_helpstring}\" FORCE)\n")
    if(_advanced)
      file(APPEND "${_output_file}" "mark_as_advanced(FORCE ${_var})\n")
    endif()
    if(_strings)
      file(APPEND "${_output_file}" "set_property(CACHE ${_var} PROPERTY STRINGS \"${_strings}\")\n")
    endif()
  endforeach()

  # TODO: Also cache *_FOUND and *_DIR variables and unset and restore them in _restore_cmake_cache?
endfunction()

function(_restore_cmake_cache _input_file)
  # Unset all cache variables!
  get_cmake_property(_cache_variables CACHE_VARIABLES)
  foreach(_var ${_cache_variables})
    unset(${_var} CACHE)
  endforeach()
  include(${_input_file})
endfunction()

if(NOT COMMAND _list_append_deduplicate)
  # append elements to a list and remove existing duplicates from the list
  # copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
  # self contained
  macro(_list_append_deduplicate listname)
    if(NOT "${ARGN}" STREQUAL "")
      if(${listname})
        list(REMOVE_ITEM ${listname} ${ARGN})
      endif()
      list(APPEND ${listname} ${ARGN})
    endif()
  endmacro()
endif()

function(_get_imported_libraries listname)
  foreach(_target ${ARGN})
    # message(STATUS "  ${_target}")
    get_target_property(_imported ${_target} IMPORTED)
    # message(STATUS "  IMPORTED: ${_imported}")
    if(_imported)
      get_target_property(_imported_configurations ${_target} IMPORTED_CONFIGURATIONS)
      # message(STATUS "  IMPORTED_CONFIGURATIONS: ${_imported_configurations}")
      if(_imported_configurations)
        string(TOUPPER "${_imported_configurations}" _imported_configurations)
        list(GET _imported_configurations 0 _configuration)
        # message(STATUS "  CONFIG: ${_configuration}")
        get_target_property(_imported_location ${_target} IMPORTED_LOCATION_${_configuration})
        # message(STATUS "  IMPORTED_LOCATION_${_configuration}: ${_imported_location}")
        if(_imported_location)
          _list_append_deduplicate(${listname} ${_imported_location})
        endif()
        get_target_property(_interface_link_libraries_config ${_target} INTERFACE_LINK_LIBRARIES_${_configuration})
        get_target_property(_interface_link_libraries ${_target} INTERFACE_LINK_LIBRARIES)
        # message(STATUS "  INTERFACE_LINK_LIBRARIES_${_configuration}: ${_interface_link_libraries_config}")
        # message(STATUS "  INTERFACE_LINK_LIBRARIES: ${_interface_link_libraries}")
        foreach(_interface_link_library ${_interface_link_libraries_config} ${_interface_link_libraries})
          if(NOT _interface_link_library)
            # Do nothing.
          elseif(IS_ABSOLUTE ${_interface_link_library})
            _list_append_deduplicate(${listname} ${_interface_link_library})
          elseif(TARGET ${_interface_link_library})
            _get_imported_libraries(${listname} ${_interface_link_library})
          else()
            find_library(${_interface_link_library}_LIBRARY ${_interface_link_library})
            if(NOT ${_interface_link_library}_LIBRARY)
              message(FATAL_ERROR "${_interface_link_library} is neither an absolute path nor another target nor the name of a library found by find_library().")
            endif()
            _list_append_deduplicate(${listname} ${${_interface_link_library}_LIBRARY})
          endif()
        endforeach()
      endif()
    endif()
  endforeach()
  set(${listname} ${${listname}} PARENT_SCOPE)
endfunction()

function(_get_imported_targets_recursive listname)
  foreach(_target ${ARGN})
    # message(STATUS "  ${_target}")
    get_target_property(_imported ${_target} IMPORTED)
    # message(STATUS "  IMPORTED: ${_imported}")
    if(_imported)
      _list_append_deduplicate(${listname} ${_target})

      get_target_property(_imported_configurations ${_target} IMPORTED_CONFIGURATIONS)
      # message(STATUS "  IMPORTED_CONFIGURATIONS: ${_imported_configurations}")
      if(_imported_configurations)
        string(TOUPPER "${_imported_configurations}" _imported_configurations)
        list(GET _imported_configurations 0 _configuration)
        # message(STATUS "  CONFIG: ${_configuration}")

        get_target_property(_interface_link_libraries_config ${_target} INTERFACE_LINK_LIBRARIES_${_configuration})
        get_target_property(_interface_link_libraries ${_target} INTERFACE_LINK_LIBRARIES)
        # message(STATUS "  INTERFACE_LINK_LIBRARIES_${_configuration}: ${_interface_link_libraries_config}")
        # message(STATUS "  INTERFACE_LINK_LIBRARIES: ${_interface_link_libraries}")
        foreach(_interface_link_library ${_interface_link_libraries_config} ${_interface_link_libraries})
          if(TARGET ${_interface_link_library})
            _get_imported_targets_recursive(${listname} ${_interface_link_library})
          endif()
        endforeach()
      endif()
    endif()
  endforeach()
  set(${listname} ${${listname}} PARENT_SCOPE)
endfunction()

# *****************************************************************************
#  User functions
# *****************************************************************************

macro(switch_to_ros2_context)
  _backup_cmake_cache(${CMAKE_CURRENT_BINARY_DIR}/cache_backup.cmake)

  # Backup and temporarily overwrite CMAKE_PREFIX_PATH, PYTHONPATH and AMENT_PREFIX_PATH in this scope.
  set(_backup_cmake_prefix_path ${CMAKE_PREFIX_PATH})
  set(_backup_python_path $ENV{PYTHONPATH})
  set(_backup_ament_prefix_path $ENV{AMENT_PREFIX_PATH})
  set(CMAKE_PREFIX_PATH ${ROS2_CMAKE_PREFIX_PATH})
  set(ENV{PYTHONPATH} ${ROS2_PYTHONPATH})
  string(REPLACE ";" ":" _ENV_AMENT_PREFIX_PATH "${ROS2_CMAKE_PREFIX_PATH}")
  set(ENV{AMENT_PREFIX_PATH} ${_ENV_AMENT_PREFIX_PATH})
  message(VERBOSE "[ros2in1_support] CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}")
  message(VERBOSE "[ros2in1_support] PYTHONPATH=$ENV{PYTHONPATH}")
  message(VERBOSE "[ros2in1_support] AMENT_PREFIX_PATH=$ENV{AMENT_PREFIX_PATH}")
endmacro()

macro(restore_ros1_context)
  # Restore CMAKE_PREFIX_PATH.
  set(CMAKE_PREFIX_PATH ${_backup_cmake_prefix_path})
  set(ENV{PYTHONPATH} ${_backup_python_path})
  set(ENV{AMENT_PREFIX_PATH} ${_backup_ament_prefix_path})
  unset(_backup_cmake_prefix_path)
  unset(_backup_python_path)
  unset(_backup_ament_prefix_path)

  _restore_cmake_cache(${CMAKE_CURRENT_BINARY_DIR}/cache_backup.cmake)
endmacro()

# Call a ROS 2 command with arguments with the ROS 2 CMake cache and environment variables.
function(call_with_ros2_context _command)
  cmake_policy(PUSH)
  cmake_minimum_required(VERSION 3.18)
  switch_to_ros2_context()

  cmake_language(CALL ${_command} ${ARGN})

  restore_ros1_context()
  cmake_policy(POP)
endfunction()

# Find a ROS 2 package in the ROS2_CMAKE_PREFIX_PATH.
# Only CMake targets and some ${_name}_* are exposed to the scope of the caller.
function(find_ros2_package _name)
  cmake_policy(PUSH)
  switch_to_ros2_context()

  # Unset some variables for common packages with conflicting names that may
  # already have been found in the ROS 1 scope. We need to find them again for ROS 2.
  foreach(_ros12_package
      ${_name}
      actionlib_msgs
      angles
      builtin_interfaces
      class_loader
      cv_bridge
      diagnostic_msgs
      geometry_msgs
      image_geometry
      image_transport
      image_geometry
      interactive_markers
      joy
      kdl_parser
      laser_geometry
      map_msgs
      message_filters
      nav_msgs
      pcl_conversions
      pcl_msgs
      pluginlib
      resource_retriever
      robot_state_publisher
      ros_environment
      rosgraph_msgs
      sensor_msgs
      shape_msgs
      statistics_msgs
      std_msgs
      std_srvs
      stereo_msgs
      tf2
      tf2_bullet
      tf2_eigen
      tf2_geometry_msgs
      tf2_kdl
      tf2_msgs
      tf2_py
      tf2_ros
      tf2_sensor_msgs
      tf2_tools
      trajectory_msgs
      unique_identifier_msgs
      urdf
      urdf_parser_plugin
      visualization_msgs)
    unset(${_ros12_package}_FOUND)
    unset(${_ros12_package}_DIR CACHE)
  endforeach()

  # Find the ROS 2 package.
  set(CMAKE_FIND_USE_PACKAGE_REGISTRY OFF)
  message(STATUS "[ros2in1_support] Trying to find ROS 2 package ${_name} within CMAKE_PREFIX_PATH \"${CMAKE_PREFIX_PATH}\"...")
  find_package(${_name} ${ARGN})
  if(${_name}_FOUND)
    message(STATUS "[ros2in1_support] Found ROS 2 package ${_name} at ${${_name}_DIR}.")
  endif()

  # # Print libraries
  # _get_imported_libraries(_imported_libraries ${${_name}_TARGETS})
  # message(STATUS "[ros2in1_support] Library dependencies of ${_name}:")
  # foreach(_lib ${_imported_libraries})
  #   message(STATUS "  - ${_lib}")
  # endforeach()

  # Make ament_index happy by installing the packages resource marker in our devel-space and install-space.
  file(COPY "${${_name}_DIR}/../../../share/ament_index/resource_index/packages/${_name}" DESTINATION "${CATKIN_DEVEL_PREFIX}/share/ament_index/resource_index/packages/${_name}")
  install(FILES "${${_name}_DIR}/../../../share/ament_index/resource_index/packages/${_name}" DESTINATION "${CATKIN_GLOBAL_SHARE_DESTINATION}/ament_index/resource_index/packages")

  # Export variables
  set(${_name}_FOUND_ROS2 ${${_name}_FOUND} PARENT_SCOPE)
  set(${_name}_TARGETS_ROS2 ${${_name}_TARGETS} PARENT_SCOPE)

  restore_ros1_context()
  cmake_policy(POP)
endfunction()

# Find and use a library from within the ROS2_CMAKE_PREFIX_PATH in ROS 1:
# - Symlink it to the ROS 1 devel-space.
# - Install it in the ROS 1 install-space.
function(find_ros2_library _name_or_location)
  # Workaround for IMPORTED_LOCATION of fastcdr library being /opt/ros/humble/lib/libfastcdr.so.1.0.24 instead of the SONAME /opt/ros/humble/lib/libfastcdr.so.1:
  string(REGEX REPLACE "\\.so\\..*" ".so" _name_or_location "${_name_or_location}")

  if(IS_ABSOLUTE "${_name_or_location}")
    set(_location "${_name_or_location}")
    get_filename_component(_name "${_name_or_location}" NAME)
  else()
    foreach(_path ${ROS2_CMAKE_PREFIX_PATH})
      list(APPEND _library_paths "${_path}/lib" "${_path}/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
    endforeach()
    unset(_location)
    find_library(_location NAMES "${_name_or_location}" PATHS ${_library_paths} NO_DEFAULT_PATH NO_CACHE)
    #message(STATUS "[ros2in1_support] find_library(_location NAMES ${_name_or_location} PATHS ${_library_paths} NO_DEFAULT_PATH NO_CACHE) -> ${_location}")
    if(NOT _location)
      message(FATAL_ERROR "[ros2in1_support] Failed to find required library ${_name_or_location}: ${_location}")
      return()
    endif()
    get_filename_component(_name "${_location}" NAME)
  endif()

  while(TRUE)
    message(STATUS "[ros2in1_support] Using ROS 2 library ${_name} at ${_location}.")

    # Symlink the library to the devel-space.
    if(CATKIN_DEVEL_PREFIX)
      file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib")
      if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/lib/${_name}")
        file(CREATE_LINK
          "${_location}" "${CATKIN_DEVEL_PREFIX}/lib/${_name}"
          SYMBOLIC
        )
      endif()
    endif()

    # Install the library.
    install(FILES ${_location} DESTINATION lib)

    # Traverse symlinks.
    if(IS_SYMLINK "${_location}")
      file(READ_SYMLINK "${_location}" _symlink_target)
      if(_symlink_target MATCHES "/")
        message(FATAL_ERROR "Only relative symlinks to another library in the same directory are supported for ROS 2 libraries.")
      endif()
      set(_name "${_symlink_target}")
      get_filename_component(_location_directory "${_location}" DIRECTORY)
      set(_location "${_location_directory}/${_symlink_target}")
    else()
      break()
    endif()
  endwhile()
endfunction()

# Given a list of imported targets, install all required ROS 2 libraries
# to the ROS 1 install-space or symlink them to the devel-space, and update the
# respective library target's IMPORTED_LOCATION property.
function(use_ros2_libraries)
  _get_imported_targets_recursive(_imported_targets ${ARGN})
  foreach(_target ${_imported_targets})
    # message(STATUS "[ros2in1_support] Processing target ${_target}...")
    get_target_property(_imported_configurations ${_target} IMPORTED_CONFIGURATIONS)
    # message(STATUS "  IMPORTED_CONFIGURATIONS: ${_imported_configurations}")
    if(_imported_configurations)
      string(TOUPPER "${_imported_configurations}" _imported_configurations)
      list(GET _imported_configurations 0 _configuration)
      # message(STATUS "  CONFIG: ${_configuration}")
      get_target_property(_imported_location ${_target} IMPORTED_LOCATION_${_configuration})
      # message(STATUS "  IMPORTED_LOCATION_${_configuration}: ${_imported_location}")
      if(_imported_location)
        find_ros2_library(${_imported_location})

        if(CATKIN_DEVEL_PREFIX)
          # Update the IMPORTED_LOCATION target property.
          get_filename_component(_library_name "${_imported_location}" NAME)
          set_target_properties(${_target} PROPERTIES IMPORTED_LOCATION_${_configuration} ${CATKIN_DEVEL_PREFIX}/lib/${_library_name})
        endif()
      endif()
    endif()

    # Export library targets by default with catkin_package().
    list(APPEND ${PROJECT_NAME}_LIBRARIES ${_target})
  endforeach()
endfunction()

function(download_ros1_bridge_nodes)
  cmake_parse_arguments(ARG "" "VERSION" "" ${ARGN})
  if(NOT ARG_VERSION)
    set(_base_url "https://github.com/Intermodalics/ros1_bridge/releases/latest/download")
  else()
    set(_base_url "https://github.com/Intermodalics/ros1_bridge/releases/download/${ARG_VERSION}")
  endif()

  if(NOT CATKIN_PACKAGE_BIN_DESTINATION)
    catkin_destinations()
  endif()

  foreach(_executable ${ARG_UNPARSED_ARGUMENTS})
    # Download and install a pre-built executable from https://github.com/Intermodalics/ros1_bridge.
    catkin_download(
      download_${_executable}
      ${_base_url}/${_executable}-${LSB_CODENAME}-${CMAKE_SYSTEM_PROCESSOR}
      REQUIRED
      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
      FILENAME ${_executable}
    )
    add_custom_command(
      TARGET download_${_executable}
      POST_BUILD
      COMMAND chmod +x ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${_executable}
    )
    set_target_properties(download_${_executable} PROPERTIES EXCLUDE_FROM_ALL FALSE)
    install(
      PROGRAMS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${_executable}
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
  endforeach()
endfunction()

# *****************************************************************************
#  Process components listed with find_package(ros2in1_support ...)
# *****************************************************************************

# Find ROS 2 packages listed as "components" in
#
#   find_package(ros2in1_support [REQUIRED] [QUIET] COMPONENTS pkg ...)
#
# .
set(ros2in1_support_TARGETS)
foreach(_comp ${ros2in1_support_FIND_COMPONENTS})
  if(ros2in1_support_FIND_REQUIRED_${_comp})
    find_ros2_package(${_comp} REQUIRED)
  elseif(ros2in1_support_FIND_QUIETLY)
    find_ros2_package(${_comp} QUIET)
  else()
    find_ros2_package(${_comp})
  endif()
  list(APPEND ros2in1_support_TARGETS ${${_comp}_TARGETS_ROS2})
endforeach()

if(ros2in1_support_TARGETS)
  message(VERBOSE "[ros2in1_support] Found the following ROS 2 targets: ${ros2in1_support_TARGETS}")
  use_ros2_libraries(${ros2in1_support_TARGETS})
endif()
