#ifndef POINT_CLOUD_GENERATOR_H
#define POINT_CLOUD_GENERATOR_H

#include <thread>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class DepthToPointCloud
{
public:
    DepthToPointCloud(ros::NodeHandle &nh);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void colorCallback(const sensor_msgs::ImageConstPtr &color_msg);
    void depthCallback(const sensor_msgs::ImageConstPtr &depth_msg);

private:
    void generatePointCloud();

    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_color_, sub_depth_;
    ros::Subscriber sub_camera_info_;
    ros::Publisher pub_pointcloud_;
    cv_bridge::CvImagePtr color_ptr_, depth_ptr_;
    double camera_fx_, camera_fy_, camera_cx_, camera_cy_, camera_factor_;
};

#endif // POINT_CLOUD_GENERATOR_H
