#include "hand_visualizer.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <vector>

template <typename T1, typename T2> bool IsEqual(T1 num1, T2 num2) {
  const double epsilon = 1e-6;
  return std::fabs(static_cast<double>(num1) - static_cast<double>(num2)) <
         epsilon;
}

std::vector<std::array<float, 2>> hand_coord = {
    {-20, -55}, {-38, -45}, {-56, -28}, {-73, -10}, {-20, -5},
    {-27, 24},  {-33, 51},  {-39, 78},  {0, 0},     {0, 25},
    {0, 50},    {0, 90},    {20, -5},   {27, 24},   {33, 51},
    {39, 78},   {37, -14},  {43, 9},    {53, 27},   {63, 44},
    {-10, -40}, {-20, -30}, {10, -60},  {24, -55},  {37, -40}};

ForceVisualizer::ForceVisualizer() {
  // Init drawing
  InitCenter();
  cv::applyColorMap(cv::Mat::zeros(1, 256, CV_8UC3), color_map_,
                    cv::COLORMAP_AUTUMN);
  // 图标中心点
  sensor_center_[kRightHand] = cv::Point(55, -50);
  sensor_center_[kLeftHand] = cv::Point(-55, -50);
  shape_[0] = 1280;
  shape_[1] = 720;
  image_center_ = cv::Point(shape_[0] / 2, shape_[1]);
  hand_k_ = 1.0 / (1.0 / kHandLimitMin - 1.0 / kHandLimitMax);
  hand_b_ = -(1.0 / kHandLimitMax) * hand_k_;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 105; j++) {
      data_[i][j] = 1;
      data_processed_[i][j] = 1;
    }
  }

  img_.create(cv::Size(shape_[0], shape_[1]), CV_8UC3);
  pub_ = n_.advertise<sensor_msgs::Image>("/hand_visual", 10);
  l_sub_ = n_.subscribe<io_msgs::touch>(

      "/touch_left_hand", 10, [this](const io_msgs::touch::ConstPtr &msg) {
        this->messageCallback(std::string("/touch_left_hand"), msg);
      });
  r_sub_ = n_.subscribe<io_msgs::touch>(
      "/touch_right_hand", 10, [this](const io_msgs::touch::ConstPtr &msg) {
        this->messageCallback(std::string("/touch_right_hand"), msg);
      });
}

ForceVisualizer::~ForceVisualizer() {}

void ForceVisualizer::InitCenter() {
  float x, y;
  for (int i = 0; i < kHandSensorNum; i++) {
    x = hand_coord[i][0], y = hand_coord[i][1];
    right_hand_center_[i] = cv::Point(-kHandSizeRatio * x, -kHandSizeRatio * y);
    left_hand_center_[i] = cv::Point(kHandSizeRatio * x, -kHandSizeRatio * y);
  }
}

void ForceVisualizer::messageCallback(std::string topic,
                                      const io_msgs::touch::ConstPtr &msg) {

  if (topic == "/touch_right_hand") {
    this->RecvTouchData(std::string("right_hand"), msg);
  } else if (topic == "/touch_left_hand") {
    this->RecvTouchData(std::string("left_hand"), msg);
  } else {
    ROS_ERROR("Received an unknown message type.");
  }
}

template <typename T>
void ForceVisualizer::RecvTouchData(std::string hand, const T &msg) {
  if (hand == "right_hand") {
    for (int i = 0; i < kDataNum; i++) {
      data_[0][i] = static_cast<int32_t>(msg->data[i]);
      data_processed_[0][i] = (1.0 / data_[0][i]) * hand_k_ + hand_b_;
    }
  } else if (hand == "left_hand") {
    for (int i = 0; i < kDataNum; i++) {
      data_[1][i] = static_cast<int32_t>(msg->data[i]);
      data_processed_[1][i] = (1.0 / data_[1][i]) * hand_k_ + hand_b_;
    }
  }
}
void ForceVisualizer::FlushImage() {
  // 切割手部数据，分割数据后绘画
  auto draw_hand = [&](int channel) {
    auto &data = data_processed_[channel];
    std::vector<float> res;
    for (int i = 0; i < kHandSensorNum; ++i) {
      res.push_back(0.01);
    }
    if (channel == kRightHand) {
      res[3] = data[0];        // thumb-1
      res[1] = data[1];        // thumb-0
      res[7] = data[2];        // index-0
      res[5] = data[3];        // index-1
      res[11] = data[4];       // middle-0
      res[9] = data[5];        // middle-1
      res[15] = data[6];       // ring-0
      res[13] = data[7];       // ring-1
      res[19] = data[8];       // pinky-0
      res[17] = data[9];       // pinky-1
      res[20] = kHandLimitMin; // hand-0
      res[24] = kHandLimitMin; // hand-1
    } else if (channel == kLeftHand) {
      res[3] = data[0];        // thumb-1
      res[1] = data[1];        // thumb-0
      res[7] = data[2];        // index-0
      res[5] = data[3];        // index-1
      res[11] = data[4];       // middle-0
      res[9] = data[5];        // middle-1
      res[15] = data[6];       // ring-0
      res[13] = data[7];       // ring-1
      res[19] = data[8];       // pinky-0
      res[17] = data[9];       // pinky-1
      res[20] = kHandLimitMin; // hand-0
      res[24] = kHandLimitMin; // hand-1
    } else if (channel == kLeftHand) {
      res[3] = data[0];        // thumb-1
      res[1] = data[1];        // thumb-0
      res[7] = data[2];        // index-0
      res[5] = data[3];        // index-1
      res[11] = data[4];       // middle-0
      res[9] = data[5];        // middle-1
      res[15] = data[6];       // ring-0
      res[13] = data[7];       // ring-1
      res[19] = data[8];       // pinky-0
      res[17] = data[9];       // pinky-1
      res[20] = kHandLimitMin; // hand-0
      res[24] = kHandLimitMin; // hand-1
    }
    return res;
  };

  cv::Mat img_draw = cv::Mat::zeros(cv::Size(shape_[0], shape_[1]), CV_8UC3);
  // 操作绘画
  std::vector<float> res;
  // Right Hand
  res = draw_hand(kRightHand);
  for (int j = 0; j < kHandSensorNum; j++) {
    if (IsEqual(res[j], 0.01)) {
      continue;
    }
    float radius = kCircleRadius * res[j] * 10;
    std::cout << "radius" << radius << std::endl;

    if (radius > kCircleRadius * 3) {
      radius = kCircleRadius * 3;
    }
    if (radius < kCircleRadius) {
      radius = kCircleRadius;
    }
    cv::Point center =
        (right_hand_center_[j] + sensor_center_[kRightHand]) * kCenterScale +
        image_center_;
    cv::circle(img_draw, center, radius,
               cv::Scalar(255, static_cast<int>(res[j] * 255), 0), -1);
  }
  // Left Hand
  res = draw_hand(kLeftHand);
  for (int j = 0; j < kHandSensorNum; j++) {
    if (IsEqual(res[j], 0.01)) {
      continue;
    }
    float radius = kCircleRadius * res[j] * 100;
    if (radius > kCircleRadius * 3) {
      radius = kCircleRadius * 3;
    }
    if (radius < kCircleRadius) {
      radius = kCircleRadius;
    }
    cv::Point center =
        (left_hand_center_[j] + sensor_center_[kLeftHand]) * kCenterScale +
        image_center_;
    cv::circle(img_draw, center, radius,
               cv::Scalar(255, static_cast<int>(res[j] * 255), 0), -1);
  }
  {
    std::lock_guard<std::mutex> lock_guard(img_mutex_);
    img_ = img_draw.clone();
  }
}

void ForceVisualizer::Run() {
  this->FlushImage();
  cv::Mat img_copy;
  {
    std::lock_guard<std::mutex> lock_guard(img_mutex_);
    img_copy = img_.clone();
  }
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_).toImageMsg();
  pub_.publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}
