# ros2in1_support
_CMake macros and functions to help finding and using ROS 2 dependencies in ROS 1 packages_

This package `ros2in1_support` allows to use both ROS 1 and ROS 2 in the same ROS 1 package.

No need to modify your ROS 2 workspace.

## Why on earth?
We got very frustrated with ros_bridge, both in terms of usability and performance... to the point where it seemed easier to just include ROS 2 headers and start working with ROS 2 directly in a ROS 1 node.

We hope it helps more gradual transitions of existing ROS 1 nodes to the ROS 2 world. There are many changes you need to do to migrate a ROS 1 package to ROS 2. This package focusses on the most important mechanisms: publish/subscribe.

We don't plan on migrating parameters or action

## Typical use: publish to ROS 2
Add this to your ROS 1 CMakeLists.txt file, after your project(myproject) function:

```cmake
# Lookup ROS 2 packages (messages and rclcpp):
find_package(ros2in1_support REQUIRED COMPONENTS
    geometry_msgs
    tf2_msgs
    # ... add other required _msgs here ...
    rclcpp
)

# If you do not set the CMAKE ROS2_SUPPORT flag, your code will build while 
# using ros2in1_support headers BUT without using ROS 2
if(ROS2_SUPPORT)
  add_definitions(-DROS2_SUPPORT)
endif()
```


In your C++ header file, add the corresponding includes:
```c++
// ROS 2 messages:
#include <ros2in1_support/conversions/geometry_msgs.h>
#include <ros2in1_support/tf.h>

// ROS 2 publisher:
#include <ros2in1_support/publisher.h>
#endif

```

In you C++ class (header), use the ros2in1_support message types:
```c++
  // ROS publishers and subscribers.
  ros::NodeHandle nh_;
  ros2in1_support::TransformBroadcaster tf_broadcaster_;
  ros2in1_support::Publisher<geometry_msgs::PointStamped> gnss_M_A_publisher_;
```

In the C++ class (implementation), initialize these objects:
```c++
  MyClass::MyClass( const ros::NodeHandle& nh )
    : tf_broadcaster_(nh_) {
    //...
    gnss_M_A_publisher_.advertise(nh_, "gnss_M_A", 10);
    //...
  }
```

In your publishing code, use the publish function as usual:
```c++
void MyClass::publishdata() {
    geometry_msgs::PointStamped point_M_A;
    //...
    gnss_M_A_publisher_.publish(point_M_A);
}
```
The above code will publish to both the ROS 1 and ROS 2 topic 'gnss_M_A' if ROS2_SUPPORT was set at cmake time.

## Typical use: subscribe to ROS 2

In order to subscribe to ROS 2 topics, your ROS 1 node will also have to launch an executor to handle the incoming data. First add this to your main() / init() function:

TODO: to we open up the im_remappings_helper too or is this not useful?

```c++
  // Initialize ROS 1
  const auto remappings = im_remappings_helper::getGlobalRemappings(argc, argv);
  ros::init(remappings, name, options);
  

#ifdef ROS2_SUPPORT
  {
    // TODO: Forwards remappings to ROS 2!
    rclcpp::InitOptions ros2_init_options;
#if ROS2_DISTRO_galactic
    ros2_init_options.shutdown_on_sigint = false;
    rclcpp::init(argc, argv, ros2_init_options);
    rclcpp::uninstall_signal_handlers();
#else
    ros2_init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, ros2_init_options, rclcpp::SignalHandlerOptions::None);
#endif
  }
#endif

  // Start global AsyncSpinner spinner for ROS 1
  std::unique_ptr<ros::AsyncSpinner> global_spinner_;
  global_spinner_.reset(new ros::AsyncSpinner(1));
  global_spinner_->start();

  // Start executor for ROS 2
#ifdef ROS2_SUPPORT
  std::unique_ptr<std::thread> ros2_executor_thread_;
  ros2_executor_thread_.reset(new std::thread([]() {
    rclcpp::Node::SharedPtr node = ros2in1_support::getRos2Node();
    rclcpp::spin(node);
  }));
#endif
```

Finally, you can subscribe using this:
```c++
  ros::Subscriber subscriber =  nh_.subscribe<sensor_msgs::NavSatFix>(
        FLAGS_gnss_topic, 10,
        [this](const sensor_msgs::NavSatFix::ConstPtr& navsat_msg) {
          this->gnssCallback(navsat_msg);
        }));
```

## Typical use: Advertise a Service
The ServiceServer has also been wrapped in order to expose it to both ROS 1 and ROS 2 nodes.

```c++
#include <ros2in1_support/service_server.h>
#include <nav_msgs/GetMap.h>

//... example implementation of GetMap Service:
bool getMapCallback(nav_msgs::GetMap::Request& /*req*/,
                                        nav_msgs::GetMap::Response& res) {
  // ...
  return true;
}

// in main() or your MapServer class:

  // Advertise the GetMap Service to ROS 1 and ROS 2
  ros2in1_support::ServiceServer<nav_msgs::GetMap> get_map_service_server_;

  get_map_service_server_.advertise(nh_, "get_map", &getMapCallback);
```

## Accessing the underlying ROS 2 API

`ros2in1_support` wraps around the ROS 2 API. If you want to access the ROS 2 API directly in your ROS 1 code, this is allowed. Just use this code:

```c++
#include "ros2in1_support/node.h"

// ...
  rclcpp::Node::SharedPtr ros2_node = getRos2Node(ros1_node_);
  ros2node->create_publisher<...>(...); // only publish to ROS 2!
```

# Limitations

There are many pitfalls on what can be done and what not in such a mixed workspace, which need to be documented or resolved:
* C++ only so far; 
* no duplicate header names and no duplicate C++ namespaces in ROS 1 and 2, e.g. no tf2_ros). 

So its usage is essentially limited to adding ROS 1 and 2 subscribers and publishers and services in the same executable, but not using any of the more advanced packages like tf, image_geometry, rviz etc.


# Roadmap / ideas

The most low-hanging fruit is probably to provide a package with some roscpp-like helpers to emulate "the missing features of ROS 1", like a central parameter server and service calls from within other ROS callbacks without blocking. That are the biggest issues when porting ROS 1 code to ROS 2. The code still needs to be ported manually then, but with this additional helpers package there is at least a functionally equivalent for ROS 2.


# Technical Implementation Details
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
for targets that link to other libraries
that may use `ros2in1_support`.

The same ROS 2 libraries may be linked and installed multiple times,
by different packages, that should not matter.
