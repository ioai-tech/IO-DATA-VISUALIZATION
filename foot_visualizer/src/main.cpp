#include "ros/init.h"
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <string>
#include "foot_visualizer.hpp"
#include "io_msgs/squashed_touch.h"
#include "io_msgs/touch.h"
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foot_visualizer");
  FootVisualizer vis;
  ros::Rate r{100};
  while (ros::ok())
  {
    vis.Publish();
    ros::spinOnce();
    r.sleep();
  }
}
