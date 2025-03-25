#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <csignal>
#include "pano2cyl.cpp"
#include <chrono>
#include <thread>
#include <iostream>

// 定义主目录和子目录
std::string BASE_DIR;
std::string RGB_DIR;
std::string PANOPTIC_DIR;
std::string PANOPTIC_CYLINDER_DIR;
std::string POINTCLOUD_DIR;

const int barWidth = 70;
std::vector<cv::Mat> panoptic_images;
std::vector<std::string> panoptic_timestamps;

void showProgressBar(float progress) {
    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

void callback(const sensor_msgs::CompressedImageConstPtr& rgb_msg,
              const sensor_msgs::CompressedImageConstPtr& panoptic_msg)
{
    try {
        // 解压缩RGB图像
        cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb_image = cv_rgb->image;

        // 解压缩深度图像
        cv_bridge::CvImagePtr cv_panoptic = cv_bridge::toCvCopy(panoptic_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat panoptic_image = cv_panoptic->image;
        // panoptic_images.push_back(panoptic_image);

        ros::Time rgb_time = rgb_msg->header.stamp;
        ros::Time panoptic_time = panoptic_msg->header.stamp;

        ros::Duration rgb_depth_diff = rgb_time - panoptic_time;

        // double max_diff = std::max({rgb_depth_diff.toSec(), rgb_cloud_diff.toSec(), depth_cloud_diff.toSec()});
        // double min_diff = std::min({rgb_depth_diff.toSec(), rgb_cloud_diff.toSec(), depth_cloud_diff.toSec()});
        // ROS_INFO("Max time difference: %.9f seconds", max_diff);
        // ROS_INFO("Min time difference: %.9f seconds", min_diff);

        // 获取当前时间戳
        std::string rgb_timestamp = std::to_string(rgb_msg->header.stamp.toNSec());
        std::string panoptic_timestamp = std::to_string(panoptic_msg->header.stamp.toNSec());
        panoptic_timestamps.push_back(panoptic_timestamp);

        // 创建目录（如果不存在）
        boost::filesystem::create_directories(RGB_DIR);
        boost::filesystem::create_directories(PANOPTIC_DIR);

        // 保存图像
        cv::imwrite(RGB_DIR + "/" + rgb_timestamp + ".jpg", rgb_image);
        cv::imwrite(PANOPTIC_DIR + "/" + panoptic_timestamp + ".jpg", panoptic_image);

        // 释放图像变量
        cv_rgb->image.release();
        cv_panoptic->image.release();

        // std::cout << RGB_DIR + "/" + rgb_timestamp + ".jpg" << std::endl;

        ROS_INFO("Data saved for timestamp: %s", panoptic_timestamp.c_str());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (pcl::IOException& e) {
        ROS_ERROR("pcl exception: %s", e.what());
    }
}

void shutdownHandler(int sig)
{
    int totalImages = panoptic_images.size();
    for (size_t i = 0; i < totalImages; ++i) {
        cv::Mat cyl_image = SphereToCylinder(panoptic_images[i], 1920, 100 * DTOR, -70 * DTOR, 30 * DTOR, -180 * DTOR, 180 * DTOR);
        std::string output_path = PANOPTIC_CYLINDER_DIR + "/" + panoptic_timestamps[i] + ".jpg";
        cv::imwrite(output_path, cyl_image);
        showProgressBar(float(i + 1) / totalImages);
    }
    std::cout << std::endl;
    ROS_INFO("All images have been processed and saved.");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_sensors");
    ros::NodeHandle nh("~");
    // signal(SIGINT, shutdownHandler);

    // Get the date from the ROS parameter server
    ros::Duration(1.0).sleep();
    std::string bag_date, dir;
    bag_date = nh.param<std::string>("date", "WRONG-DIR");
    dir = nh.param<std::string>("dir", "WRONG-DIR");

    // Define directories based on the date parameter
    BASE_DIR = dir + "/" + bag_date;
    RGB_DIR = BASE_DIR + "/zed_images";
    PANOPTIC_DIR = BASE_DIR + "/theta_images";
    PANOPTIC_CYLINDER_DIR = BASE_DIR + "/theta_cylinder_images";
    ROS_INFO("BASE_DIR: %s", BASE_DIR.c_str());

    // Create directories if they do not exist
    if (!boost::filesystem::exists(BASE_DIR)) {
        boost::filesystem::create_directories(BASE_DIR);
    }
    if (!boost::filesystem::exists(RGB_DIR)) {
        boost::filesystem::create_directories(RGB_DIR);
    }
    if (!boost::filesystem::exists(PANOPTIC_DIR)) {
        boost::filesystem::create_directories(PANOPTIC_DIR);
    }
    if (!boost::filesystem::exists(PANOPTIC_CYLINDER_DIR)) {
        boost::filesystem::create_directories(PANOPTIC_CYLINDER_DIR);
    }

    message_filters::Subscriber<sensor_msgs::CompressedImage> rgb_sub(nh, "/zed2/zed_node/rgb/image_rect_color/compressed", 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> depth_sub(nh, "/camera/color/image_raw/compressed", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(15), rgb_sub, depth_sub);
    // 创建目录（如果不存在）
    if (!boost::filesystem::exists(RGB_DIR)) {
        boost::filesystem::create_directories(RGB_DIR);
    }
    if (!boost::filesystem::exists(PANOPTIC_DIR)) {
        boost::filesystem::create_directories(PANOPTIC_DIR);
    }
    if (!boost::filesystem::exists(PANOPTIC_CYLINDER_DIR)) {
        boost::filesystem::create_directories(PANOPTIC_CYLINDER_DIR);
    }

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ROS_INFO("SYNCHRONIZER REGISTERED.");
    ros::spin();


    return 0;
}
