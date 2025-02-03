# ros2in1_support
_CMake macros and functions to help finding and using ROS 2 dependencies in ROS 1 packages_

This package `ros2in1_support`
with CMake functions `find_ros2_package()` and `use_ros2_libraries()`
and some related helper macros and functions
allows to use both ROS 1 and ROS 2 in the same package.
The main problem here that forbids to simply add a ROS 1 and ROS 2 underlay
to the `CMAKE_PREFIX_PATH` at the same time
is that some packages exist in both ROS versions,
like `std_msgs` or `tf2_ros`.
So we need to keep track of them separately, in two separate CMake caches.

The CMake cache is backed up before running `find_ros2_package()`,
then CMake variables like `CMAKE_PREFIX_PATH`
or environment variables like `PYTHONPATH`
are redirected to the ROS 2 underlay (usually `/opt/ros/<ROS2_DISTRO>`),
then we call `find_package()` as usual
which populates the cache and creates imported targets,
and then the original cache contents are restored.
At the moment the cache is not preserved in between calls,
but that should be possible to prevent
having to find some packages multiple times if they are common dependencies.

This approach only works because back in the ROS 1 world
we can use the imported targets for ROS 2 packages only,
where the [target properties](https://cmake.org/cmake/help/latest/manual/cmake-properties.7.html#properties-on-targets)
have all the required information on include directories
([`INTERFACE_INCLUDE_DIRECTORIES`](https://cmake.org/cmake/help/latest/prop_tgt/INTERFACE_INCLUDE_DIRECTORIES.html))
or transitive linking dependencies ([`INTERFACE_LINK_LIBRARIES`](https://cmake.org/cmake/help/latest/prop_tgt/INTERFACE_LINK_LIBRARIES.html))
and the absolute location of libraries ([`IMPORTED_LOCATION`](https://cmake.org/cmake/help/latest/prop_tgt/IMPORTED_LOCATION.html)),
while in ROS 1 packages we typically do not create targets with
potentially conflicting names and export their stuff by setting variables
like `<pkg>_INCLUDE_DIRECTORIES` and `<pkg>_LIBRARIES` only.
If there are targets, then mostly for system libraries which are the same
for ROS 1 and 2 on the same platform (in the same Docker container).

Additionally, the function `use_ros2_libraries()` symlinks
all required ROS 2 libraries by the given targets to the ROS 1 devel-space,
adds install rules to copy them over to the ROS 1 install-space
(if the workspace is configured accordingly),
and redirects the `IMPORTED_LOCATION` property of the respective targets
to get the embedded `RUNPATH` (formerly `RPATH`) right.
Without, that would have caused linker errors down the line
for targets that link to `librovioli_lib.so` and other libraries
that may use `ros2in1_support`.
The same ROS 2 libraries may be linked and installed multiple times,
by different packages, that should not matter.
