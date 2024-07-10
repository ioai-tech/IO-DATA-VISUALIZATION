#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

// 全局变量
image_transport::Publisher pubImage;
int numDisparities = 16; // 根据需要调整此值
cv::Mat depth_pic;

// 将深度图像转换为视差图的函数
void ConvertToDisparity(const cv::Mat &src, cv::Mat &dst, int maxd)
{
    CV_Assert(src.depth() == CV_16U);
    cv::Mat tmp;
    if (src.channels() == 3)
        cv::cvtColor(src, tmp, cv::COLOR_BGR2GRAY);
    else
        tmp = src;
    const float denom = 1.f / 256;
    maxd = 256 * maxd + 1;

    dst.create(src.size(), CV_32F);
    for (int y = 0; y < dst.rows; y++)
    {
        const ushort *srcptr = tmp.ptr<ushort>(y);
        float *dstptr = dst.ptr<float>(y);
        for (int x = 0; x < dst.cols; x++)
        {
            const ushort p = srcptr[x];
            dstptr[x] = p > 0 && p < maxd ? denom * (p - 1) : -1.f;
        }
    }
}

// 深度图像的回调函数
void DepthCallback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    cv_bridge::CvImagePtr depth_ptr;
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    depth_pic = depth_ptr->image;

    cv::Mat disparity, disparityColor;
    ConvertToDisparity(depth_pic, disparity, numDisparities);
    disparity.convertTo(disparityColor, CV_8U, 255. / numDisparities);
    cv::applyColorMap(disparityColor, disparityColor, cv::COLORMAP_JET);
    disparityColor.setTo(cv::Scalar::all(0), disparity < 0);

    cv_bridge::CvImage out_msg;
    out_msg.header = depth_msg->header; // 保持时间戳一致性
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = disparityColor;
    pubImage.publish(out_msg.toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ColouringDepthImage");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ROS_INFO("The depth_image_colourizer node started!");

    image_transport::Subscriber sub1 = it.subscribe("/realsense/aligned_depth_to_color/image_raw", 1, DepthCallback);
    pubImage = it.advertise("/depth/image_coloured", 1);

    ros::spin();
    return 0;
}
