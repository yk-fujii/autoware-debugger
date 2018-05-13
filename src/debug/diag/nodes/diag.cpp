#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diag");
  pub = nh_.advertise<std_msgs::Int32>("diag_tester", 10, false);

  std_msgs::Int32 count;
  count.data = 0;
  while (ros::ok())
  {
    count.data++;
    pub.publish(count);
    ros::spinOnce();
  }
  return 0;
}
