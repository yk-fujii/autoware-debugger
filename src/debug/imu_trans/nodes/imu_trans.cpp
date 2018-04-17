#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <unordered_map>

#include <autoware_msgs/CanInfo.h>
#include <geometry_msgs/TwistStamped.h>

std::unordered_map<std::string, ros::Subscriber> Subs;
ros::Publisher pub;

float cur_vel = 0.0;

void doTransform(const sensor_msgs::Imu &imu_in)
{
#if 1
  sensor_msgs::Imu imu_out = imu_in;
  double r, p, y;
  tf::Quaternion imu_qu(imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z, imu_in.orientation.w);

  tf::Matrix3x3(imu_qu).getRPY(r, p, y);
  r = 0.0;
  //	p = 0.0;
  y = 0.0;

  imu_qu.setRPY(r, -p, y);

  Eigen::Quaternion<double> qr(imu_qu.w(), imu_qu.x(), imu_qu.y(), imu_qu.z());

  // Discard translation, only use orientation for IMU transform
  // Eigen::Quaternion<double> r(imu_in.orientation.w,
  //		imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
  Eigen::Transform<double, 3, Eigen::Affine> t(qr);
  Eigen::Vector3d accel =
      t * Eigen::Vector3d(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);

  imu_out.linear_acceleration.x = accel.x();
  imu_out.linear_acceleration.y = accel.y();
  imu_out.linear_acceleration.z = accel.z();

  static double init_time = 0.0;
  double time = imu_in.header.stamp.sec + (imu_in.header.stamp.nsec * 1e-9);
  if (!init_time)
  {
    init_time = time;
  }
  pub.publish(imu_out);
// fprintf(stderr, "%f,%f,%f\n", time - init_time, accel.x() / 9.80665, cur_vel);

#else
  fprintf(stderr, "%f,%f\n", imu_in.linear_acceleration.x, imu_in.linear_acceleration.y);
#endif
  //	std::cout << accel.x() << "," << accel.y() << "," << accel.z() << std::endl;
}

void callbackCanInfo(const autoware_msgs::CanInfo &msg)
{
  cur_vel = msg.speed;
}

void callbackImu(const sensor_msgs::Imu &msg)
{
  doTransform(msg);
}

void callbackCurVel(const geometry_msgs::TwistStamped &msg)
{
  cur_vel = msg.twist.linear.x * 3.6;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ims");
  ros::NodeHandle nh_;
  fprintf(stderr, "time-sec, nsec, speed, acc_g\n");
  Subs["imu_raw"] = nh_.subscribe("imu_raw", 10, callbackImu);
  Subs["can_info"] = nh_.subscribe("can_info", 10, callbackCanInfo);
  Subs["curvel"] = nh_.subscribe("current_velocity", 10, callbackCurVel);
  pub = nh_.advertise<sensor_msgs::Imu>("imu_trans", 10, false);
  ros::spin();
  return 0;
}
