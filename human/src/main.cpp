#include "ros/init.h"
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <string>
#include "human.hpp"
#include "io_msgs/squashed_touch.h"
#include "io_msgs/touch.h"
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human");
  Human human;
  ros::Rate r{100};
  while (ros::ok())
  {
    human.Publish();
    ros::spinOnce();
    r.sleep();
  }
}
