#include <ros/ros.h>
#include "AutolivNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autoliv");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  octopus::AutolivNode n(node, priv_nh);
  ros::spin();

  return 0;
}