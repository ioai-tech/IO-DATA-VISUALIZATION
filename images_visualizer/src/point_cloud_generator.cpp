#include "point_cloud_generator.h"

DepthToPointCloud::DepthToPointCloud(ros::NodeHandle &nh) : it_(nh)
{
    sub_camera_info_ = nh.subscribe("/realsense/color/camera_info", 1, &DepthToPointCloud::cameraInfoCallback, this);
    sub_color_ = it_.subscribe("/realsense/color/image_raw", 1, &DepthToPointCloud::colorCallback, this);
    sub_depth_ = it_.subscribe("/realsense/aligned_depth_to_color/image_raw", 1, &DepthToPointCloud::depthCallback, this);
    pub_pointcloud_ = nh.advertise<PointCloud>("/generated_point_cloud", 1);
}

void DepthToPointCloud::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    camera_fx_ = msg->K[0];
    camera_fy_ = msg->K[4];
    camera_cx_ = msg->K[2];
    camera_cy_ = msg->K[5];
    camera_factor_ = 0.001;
}

void DepthToPointCloud::colorCallback(const sensor_msgs::ImageConstPtr &color_msg)
{
    try
    {
        color_ptr_ = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
    }
}

void DepthToPointCloud::depthCallback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    try
    {
        depth_ptr_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        generatePointCloud();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
}

void DepthToPointCloud::generatePointCloud()
{
    if (!color_ptr_ || !depth_ptr_)
        return;

    int rows = color_ptr_->image.rows;
    int cols = color_ptr_->image.cols;

    PointCloud::Ptr cloud(new PointCloud());
    cloud->height = 1;
    cloud->width = cols * rows;
    cloud->points.reserve(cloud->width); // 预分配内存
    cloud->is_dense = false;

#pragma omp parallel for
    for (int idx = 0; idx < cloud->width; ++idx)
    {
        int m = idx / cols;
        int n = idx % cols;

        ushort d = depth_ptr_->image.at<ushort>(m, n);
        if (d == 0)
            continue;

        pcl::PointXYZRGB p;
        p.z = d * camera_factor_;
        p.x = (n - camera_cx_) * p.z / camera_fx_;
        p.y = (m - camera_cy_) * p.z / camera_fy_;

        cv::Vec3b color = color_ptr_->image.at<cv::Vec3b>(m, n);
        p.r = color[2];
        p.g = color[1];
        p.b = color[0];

#pragma omp critical
        cloud->points.emplace_back(p); // 在临界区内添加点
    }

    // 异步处理点云数据
    std::thread processing_thread([cloud, this]()
                                  {
        PointCloud::Ptr cloud_filtered(new PointCloud());
        // 降采样
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f); // 设置体素大小
        vg.filter(*cloud_filtered);

        // 直通滤波器 - X方向
        pcl::PassThrough<pcl::PointXYZRGB> pass_x;
        pass_x.setInputCloud(cloud_filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-2, 2); // X方向的范围，根据需求调整
        pass_x.filter(*cloud_filtered);

        // 直通滤波器 - Y方向
        pcl::PassThrough<pcl::PointXYZRGB> pass_y;
        pass_y.setInputCloud(cloud_filtered);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-2, 2); // Y方向的范围，根据需求调整
        pass_y.filter(*cloud_filtered);

        // 直通滤波器 - Z方向
        pcl::PassThrough<pcl::PointXYZRGB> pass_z;
        pass_z.setInputCloud(cloud_filtered);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.1, 6.0); // Z方向的范围，根据需求调整
        pass_z.filter(*cloud_filtered);

        // 检查点云是否为空
        if (cloud->points.empty())
        {
            ROS_WARN("Filtered point cloud is empty after pass-through filtering.");
            return;
        }

        // // 去除离群点
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        // sor.setInputCloud(cloud);
        // sor.setMeanK(50);
        // sor.setStddevMulThresh(1.0);
        // sor.filter(*cloud_filtered);

        // // 再次检查点云是否为空
        // if (cloud_filtered->points.empty())
        // {
        //     ROS_WARN("Filtered point cloud is empty after outlier removal.");
        //     return;
        // }

        // // 平滑处理
        // pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
        // mls.setInputCloud(cloud);
        // mls.setComputeNormals(false);
        // mls.setPolynomialOrder(2);
        // mls.setSearchRadius(0.03);
        // mls.process(*cloud_filtered);

        cloud_filtered->header.frame_id = "link_Realsense_Camera";
        pcl_conversions::toPCL(ros::Time::now(), cloud_filtered->header.stamp);

        // 发布点云之前再次检查是否为空
        if (cloud_filtered->points.empty())
        {
            ROS_WARN("Final point cloud is empty, not publishing.");
            return;
        }

        try
        {
            pub_pointcloud_.publish(cloud_filtered);
        }
        catch (const ros::serialization::StreamOverrunException &e)
        {
            ROS_ERROR("Stream Overrun Exception: %s", e.what());
        } });

    processing_thread.detach(); // 将线程分离，允许它独立运行
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_generator");
    ros::NodeHandle nh;
    ROS_INFO("The point_cloud_generator started!");

    DepthToPointCloud converter(nh);
    ros::spin();

    return 0;
}
