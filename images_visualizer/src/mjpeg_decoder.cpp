#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

image_transport::Publisher image_pub;
cv::Mat ori_img;

void MjpegCallback(const sensor_msgs::CompressedImage &commpressed_img) {
  ori_img = cv::imdecode(
      cv::Mat(1, commpressed_img.data.size(), CV_8UC1, (uint8_t *)commpressed_img.data.data()),
      cv::IMREAD_UNCHANGED);

  cv_bridge::CvImage out_msg;
  out_msg.header = commpressed_img.header; // 保持时间戳一致性
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = ori_img;
  image_pub.publish(out_msg.toImageMsg());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mjpeg_decoder");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  ros::Subscriber sub1 =
      nh.subscribe("mjpeg_raw", 1, MjpegCallback);
  image_pub = it.advertise("image_decoded", 1);

  ros::spin();
  return 0;
}
