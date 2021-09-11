#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <vehicle_simulator/MecanumSensor.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define a 0.228
#define b 0.158
#define r 0.045

using vehicle_simulator::MecanumSensor;
using nav_msgs::Odometry;

ros::Publisher odom_pub;

void sensorCb(MecanumSensor msg)
{
  static ros::Time prev_time = msg.header.stamp;
  static double pos_x = 0;
  static double pos_y = 0;
  static double odom_gain_x = 1.0;
  static double odom_gain_y = 1.0;

  ros::Time t = msg.header.stamp;
  double dt = (t - prev_time).toSec();
  double vel1 = msg.vel1;
  double vel2 = msg.vel2;
  double vel3 = msg.vel3;
  double vel4 = msg.vel4;
  double roll, pitch, yaw;
  tf2::Quaternion q;
  tf2::fromMsg(msg.orientation, q);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  double vx_body = odom_gain_x * (vel1 + vel2 + vel3 + vel4) * r / 4;
  double vy_body = odom_gain_y * (vel1 + vel3 - vel2 - vel4) * r / 4;

  double vx_ENU = vx_body * cos(yaw) - vy_body * sin(yaw);
  double vy_ENU = vx_body * sin(yaw) + vy_body * cos(yaw);

  static double prev_vx_ENU = vx_ENU;
  static double prev_vy_ENU = vy_ENU;
  pos_x += ((vx_ENU + prev_vx_ENU) / 2) * dt;
  pos_y += ((vy_ENU + prev_vy_ENU) / 2) * dt;

  Odometry odom;
  odom.header.stamp = msg.header.stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = pos_x;
  odom.pose.pose.position.y = pos_y;
  odom.pose.pose.position.z = 0.0;
  odom.twist.twist.linear.x = vx_body;
  odom.twist.twist.linear.y = vy_body;
  odom.twist.twist.linear.z = 0.0;
  odom.pose.pose.orientation = msg.orientation;
  odom.twist.twist.angular = msg.angular_velocity;
  odom_pub.publish(odom);

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = msg.header.stamp;
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = pos_x;
  transformStamped.transform.translation.y = pos_y;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation = msg.orientation;
  br.sendTransform(transformStamped);

  prev_time = t;
  prev_vx_ENU = vx_ENU;
  prev_vy_ENU = vy_ENU;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle nh;
  ros::Subscriber sensor_sub = nh.subscribe<MecanumSensor>("mecanum_sensor", 1, sensorCb);
  odom_pub = nh.advertise<Odometry>("odometry", 1);
  ros::spin();
  return 0;
}
