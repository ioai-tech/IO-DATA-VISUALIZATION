#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "foot_tf_broadcaster");

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transform_stamped;

  static_transform_stamped.header.stamp = ros::Time::now();
  static_transform_stamped.header.frame_id = "world";
  static_transform_stamped.child_frame_id = "base_link";
  static_transform_stamped.transform.translation.x = 0.0;
  static_transform_stamped.transform.translation.y = 0.0;
  static_transform_stamped.transform.translation.z = 0.0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);  // Roll, pitch, yaw
  static_transform_stamped.transform.rotation.x = quat.x();
  static_transform_stamped.transform.rotation.y = quat.y();
  static_transform_stamped.transform.rotation.z = quat.z();
  static_transform_stamped.transform.rotation.w = quat.w();

  static_broadcaster.sendTransform(static_transform_stamped);

  ros::spin();
  return 0;
}
