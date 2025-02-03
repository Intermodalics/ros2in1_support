/******************************************************************************
 *  Copyright (c) 2024, Intermodalics BVBA                                    *
 *  All rights reserved.                                                      *
 ******************************************************************************/

#include "ros2in1_support/node.h"

#ifdef ROS2_SUPPORT

namespace ros2in1_support {

rclcpp::Node::SharedPtr getRos2Node(const std::string& ns) {
  static std::mutex ros2_nodes_mutex;
  static rclcpp::Node::SharedPtr ros2_root_node;
  static std::map<std::string, rclcpp::Node::SharedPtr> ros2_nodes;
  std::lock_guard<std::mutex> lock(ros2_nodes_mutex);
  auto& ros2_node = ros2_nodes[ns];
  if (!ros2_node) {
    if (!ros2_root_node) {
      std::string node_name = ros::this_node::getName();
      node_name = node_name.substr(node_name.find_last_of('/') + 1);
      ros2_root_node = std::make_shared<rclcpp::Node>(node_name);
    }
    if (ns.empty() || (ns == "/")) {
      ros2_node = ros2_root_node;
    } else {
      ros2_node = ros2_root_node->create_sub_node(ns.substr(1));
    }
  }
  return ros2_node;
}

rclcpp::Node::SharedPtr getRos2Node(const ros::NodeHandle& ros1_node) {
  return getRos2Node(ros1_node.getNamespace());
}

rclcpp::Node::SharedPtr getRos2Node() {
  return getRos2Node(ros::this_node::getNamespace());
}

#else  // ROS2_SUPPORT

namespace ros2in1_support {

std::shared_ptr<rclcpp::Node> getRos2Node(const std::string&) {
  return nullptr;
}

std::shared_ptr<rclcpp::Node> getRos2Node(const ros::NodeHandle&) {
  return nullptr;
}

std::shared_ptr<rclcpp::Node> getRos2Node() {
  return nullptr;
}

#endif  // ROS2_SUPPORT

}  // namespace ros2in1_support
