#ifndef SOFTENABLE_BT__TF_LISTENER_WRAPPER_HPP_
#define SOFTENABLE_BT__TF_LISTENER_WRAPPER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace softenable_bt
{

class TFListenerWrapper
{
public:
  explicit TFListenerWrapper(const rclcpp::Node::SharedPtr& node);

  tf2_ros::Buffer& getBuffer();
  rclcpp::Node::SharedPtr getNode() const;
  geometry_msgs::msg::PoseStamped lookupTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Duration& timeout = rclcpp::Duration::from_seconds(1.0)
  );


private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace softenable_bt

#endif  // SOFTENABLE_BT__TF_LISTENER_WRAPPER_HPP_
