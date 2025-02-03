/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#ifndef ROS2IN1_SUPPORT_NODE_H
#define ROS2IN1_SUPPORT_NODE_H

#include <map>
#include <mutex>
#include <string>

#include <ros/node_handle.h>
#include <ros/this_node.h>

#ifdef ROS2_SUPPORT
#include <rclcpp/node.hpp>

namespace ros2in1_support {

rclcpp::Node::SharedPtr getRos2Node(const std::string& ns);
rclcpp::Node::SharedPtr getRos2Node(const ros::NodeHandle& ros1_node);
rclcpp::Node::SharedPtr getRos2Node();

#else  // ROS2_SUPPORT

namespace rclcpp {
class Node;
}

namespace ros2in1_support {

std::shared_ptr<rclcpp::Node> getRos2Node(const std::string&);
std::shared_ptr<rclcpp::Node> getRos2Node(const ros::NodeHandle&);
std::shared_ptr<rclcpp::Node> getRos2Node();

#endif  // ROS2_SUPPORT

}  // namespace ros2in1_support

#endif  // ROS2IN1_SUPPORT_NODE_H
