#include "softenable_bt/helpers/tf_listener_wrapper.hpp"

namespace softenable_bt
{

TFListenerWrapper::TFListenerWrapper(const rclcpp::Node::SharedPtr& node)
: node_(node),
  tf_buffer_(node_->get_clock()),
  tf_listener_(tf_buffer_)
{
}

tf2_ros::Buffer& TFListenerWrapper::getBuffer()
{
  return tf_buffer_;
}

rclcpp::Node::SharedPtr TFListenerWrapper::getNode() const
{
  return node_;
}

geometry_msgs::msg::PoseStamped TFListenerWrapper::lookupTransform(
  const std::string& target_frame,
  const std::string& source_frame,
  const rclcpp::Duration& timeout)
{
  if (!tf_buffer_.canTransform(
        target_frame,
        source_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(timeout.seconds())))
  {
    throw tf2::TransformException(
      "Transform from '" + source_frame + "' to '" + target_frame + "' not available after " +
      std::to_string(timeout.seconds()) + " seconds.");
  }

  geometry_msgs::msg::TransformStamped transform_stamped =
    tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = transform_stamped.header;
  pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
  pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
  pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
  pose_stamped.pose.orientation = transform_stamped.transform.rotation;

  return pose_stamped;
}

}  // namespace softenable_bt
