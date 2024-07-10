#ifndef RVIZ_FOOT_VISUALIZER_SRC_FOOT_VISUALIZER_SRC_FOOT_VISUALIZER_HPP_
#define RVIZ_FOOT_VISUALIZER_SRC_FOOT_VISUALIZER_SRC_FOOT_VISUALIZER_HPP_
#include <array>
#include <cstdint>
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include "io_msgs/squashed_touch.h"
#include "io_msgs/touch.h"
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

class FootVisualizer
{
public:
  FootVisualizer()
  {
    cnt_ = 0;
    ros::NodeHandle n;
    haptic_lf_sub_ = n.subscribe<io_msgs::touch>(
        "/touch_left_foot", 10, [this](const io_msgs::touch::ConstPtr &msg)
        { this->LeftFootCallback(msg); });
    haptic_rf_sub_ = n.subscribe<io_msgs::touch>(
        "/touch_right_foot", 10, [this](const io_msgs::touch::ConstPtr &msg)
        { this->RightFootCallback(msg); });
    haptic_sub_ = n.subscribe<io_msgs::squashed_touch>(
        "/touch_data", 10, [this](const io_msgs::squashed_touch::ConstPtr &msg)
        { this->TouchDataCallback(msg); });
    marker_pub_ = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker", 1);
  }
  visualization_msgs::Marker GetFoot()
  {
    visualization_msgs::Marker marker;
    marker.header.seq = cnt_++;
    marker.header.stamp = ros::Time();
    marker.header.frame_id = "base_link";
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 25;
    marker.scale.y = 25;
    marker.scale.z = 25;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.mesh_resource = "package://foot_visualizer/meshes/foot.stl";
    return marker;
  }
  void Publish()
  {
    visualization_msgs::MarkerArray marker_array;
    if (!left_foot_.empty())
    {
      for (int i = 0; i < left_foot_.size(); i++)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "link_LeftFoot_R";
        marker.header.stamp = ros::Time();
        marker.ns = "foot_force";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = right_coord_[i].first * 0.1;
        marker.pose.position.y = (right_coord_[i].second + 3.5) * 0.1;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = -0.707;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0.707;
        marker.scale.x = (left_foot_[i] * k_ + b_) * 0.1;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 255;
        marker.color.g = left_foot_[i] * color_k_ + color_b_;
        marker.color.b = 0.0;
        // only if using a MESH_RESOURCE marker type:
        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        marker_array.markers.push_back(marker);
      }
    }
    if (!right_foot_.empty())
    {
      for (int i = 0; i < right_foot_.size(); i++)
      {
        if (right_foot_[i] == 0)
        {
          continue;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "link_RightFoot_R";
        marker.header.stamp = ros::Time();
        marker.ns = "foot_force";
        marker.id = 99 + i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (-right_coord_[i].first) * 0.1;
        marker.pose.position.y = (right_coord_[i].second + 3.5) * 0.1;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = -0.707;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0.707;
        marker.scale.x = (right_foot_[i] * k_ + b_) * 0.1;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 255;
        marker.color.g = right_foot_[i] * color_k_ + color_b_;
        marker.color.b = 0.0;
        // only if using a MESH_RESOURCE marker type:
        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        marker_array.markers.push_back(marker);
      }
    }
    // marker_array.markers.push_back(GetFoot());
    marker_pub_.publish(marker_array);
  }

private:
  // raw scale:[4095,2000], arror lenth:[0,4]
  float k_ = -0.001909;
  float b_ = 7.817355;
  float color_k_ = 0.232876;
  float color_b_ = -698.630136;
  std::vector<float> DrawFoot(std::array<int16_t, 105> data)
  {
    // for (int i = 0; i < 105; i++) std::cout << data[i] << " ";
    // std::cout << std::endl;
    std::vector<float> res;
    for (int i = 0; i < 15; i++)
    {
      switch (i)
      {
      case 0:
        for (int j = 1; j < 5; j++)
        {
          res.push_back(data[i * 7 + j]);
        }
        break;
      case 1:
        for (int j = 0; j < 6; j++)
        {
          res.push_back(data[i * 7 + j]);
        }
        break;
      case 14:
        for (int j = 1; j < 6; j++)
        {
          res.push_back(data[i * 7 + j]);
        }
        break;
      default:
        for (int j = 0; j < 7; j++)
        {
          res.push_back(data[i * 7 + j]);
        }
        break;
      }
    }
    return res;
  }
  int cnt_ = 0;
  void LeftFootCallback(const io_msgs::touch::ConstPtr &msg)
  {
    ROS_INFO("lf detected");
    std::array<int16_t, 105> data;
    std::copy(msg->data.begin(), msg->data.end(), data.begin());
    left_foot_ = DrawFoot(data);
  }
  void RightFootCallback(const io_msgs::touch::ConstPtr &msg)
  {
    ROS_INFO("lf detected");
    std::array<int16_t, 105> data;
    std::copy(msg->data.begin(), msg->data.end(), data.begin());
    right_foot_ = DrawFoot(data);
  }
  void TouchDataCallback(const io_msgs::squashed_touch::ConstPtr &msg)
  {
    for (auto &it : msg->data)
    {
      std::array<int16_t, 105> data;
      std::copy(it.data.begin(), it.data.end(), data.begin());
      if (it.header.frame_id == "left_foot")
      {
        left_foot_ = DrawFoot((data));
      }
      if (it.header.frame_id == "right_foot")
      {
        right_foot_ = DrawFoot((data));
      }
    }
  }
  ros::Subscriber haptic_lf_sub_;
  ros::Subscriber haptic_rf_sub_;
  ros::Subscriber haptic_sub_;
  ros::Publisher marker_pub_;
  std::vector<std::pair<float, float>> right_coord_ = {
      {-0.0, -0.0}, {-0.33, 0.02}, {-0.62, 0.02}, {-0.9, -0.0}, {0.25, -0.41}, {-0.05, -0.42}, {-0.35, -0.42}, {-0.67, -0.42}, {-0.97, -0.42}, {-1.25, -0.42}, {0.35, -0.84}, {0.05, -0.84}, {-0.26, -0.84}, {-0.58, -0.84}, {-0.88, -0.84}, {-1.2, -0.87}, {-1.5, -0.88}, {0.38, -1.29}, {0.05, -1.28}, {-0.29, -1.29}, {-0.62, -1.29}, {-0.96, -1.29}, {-1.3, -1.29}, {-1.62, -1.3}, {0.34, -1.72}, {0.01, -1.72}, {-0.33, -1.72}, {-0.67, -1.72}, {-1.02, -1.72}, {-1.37, -1.72}, {-1.69, -1.75}, {0.27, -2.16}, {-0.04, -2.16}, {-0.38, -2.16}, {-0.71, -2.16}, {-1.05, -2.16}, {-1.39, -2.16}, {-1.71, -2.16}, {0.18, -2.62}, {-0.12, -2.62}, {-0.45, -2.62}, {-0.77, -2.62}, {-1.08, -2.62}, {-1.41, -2.62}, {-1.71, -2.62}, {0.02, -3.09}, {-0.25, -3.09}, {-0.54, -3.09}, {-0.83, -3.09}, {-1.13, -3.09}, {-1.43, -3.09}, {-1.71, -3.09}, {-0.06, -3.6}, {-0.33, -3.6}, {-0.62, -3.6}, {-0.89, -3.6}, {-1.17, -3.6}, {-1.45, -3.6}, {-1.71, -3.6}, {-0.08, -4.15}, {-0.36, -4.15}, {-0.62, -4.15}, {-0.91, -4.15}, {-1.17, -4.15}, {-1.45, -4.15}, {-1.72, -4.15}, {-0.1, -4.7}, {-0.37, -4.7}, {-0.63, -4.7}, {-0.91, -4.7}, {-1.17, -4.7}, {-1.45, -4.7}, {-1.71, -4.7}, {-0.12, -5.25}, {-0.38, -5.25}, {-0.65, -5.25}, {-0.91, -5.25}, {-1.17, -5.25}, {-1.44, -5.25}, {-1.71, -5.25}, {-0.14, -5.83}, {-0.4, -5.83}, {-0.65, -5.83}, {-0.91, -5.83}, {-1.17, -5.83}, {-1.42, -5.83}, {-1.68, -5.83}, {-0.19, -6.39}, {-0.42, -6.39}, {-0.67, -6.4}, {-0.91, -6.4}, {-1.15, -6.42}, {-1.39, -6.42}, {-1.62, -6.42}, {-0.38, -6.9}, {-0.65, -7.0}, {-0.9, -7.0}, {-1.13, -7.0}, {-1.41, -6.92}};

  std::vector<float> left_foot_;
  std::vector<float> right_foot_;
};

#endif // RVIZ_FOOT_VISUALIZER_SRC_FOOT_VISUALIZER_SRC_FOOT_VISUALIZER_HPP_
