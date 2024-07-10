#ifndef FORCE_VISUALIZER_H_
#define FORCE_VISUALIZER_H_

#include "ros/node_handle.h"
#include "ros/ros.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <topic_tools/shape_shifter.h>

#include "io_msgs/touch.h"
#include "io_msgs/old_touch.h"

enum DataType { kRightHand, kLeftHand, kRightFoot, kLeftFoot };

constexpr int32_t kBufferSize = 432;
constexpr int32_t kHandSensorNum = 25;
constexpr int32_t kHandLimitMax = 65535;
constexpr int32_t kHandLimitMin = 1600;
constexpr float kHandSizeRatio = 0.5;
constexpr int32_t kCircleRadius = 9;
constexpr int32_t kCenterScale = 6;
constexpr int32_t kDataNum = 105;

class ForceVisualizer {
public:
  ~ForceVisualizer();
  explicit ForceVisualizer();
  void messageCallback(std::string topic, const io_msgs::touch::ConstPtr &msg);
  template <typename T>
  void RecvTouchData(std::string hand, const T &msg);
  void FlushImage(); // 重画图像
  void Run();

private:
  void InitCenter();
  int32_t shape_[2];
  cv::Mat color_map_;
  cv::Mat img_;
  cv::Point image_center_;
  cv::Point sensor_center_[4];
  cv::Point left_hand_center_[kHandSensorNum];
  cv::Point right_hand_center_[kHandSensorNum];
  int32_t data_[4][kDataNum];
  float data_processed_[4][kDataNum];
  float hand_k_; // kx+b
  float hand_b_;
  int32_t data_type_;

  // Locks
  std::mutex data_mutex_;
  std::mutex img_mutex_;

  // ROS
  ros::Publisher pub_;
  ros::NodeHandle n_;
  ros::Subscriber l_sub_;
  ros::Subscriber r_sub_;
};

#endif // FORCE_VISUALIZER_H_
