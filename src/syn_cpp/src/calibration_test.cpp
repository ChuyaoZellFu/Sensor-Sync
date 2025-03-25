#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class SyncSensors
{
public:
    SyncSensors(ros::NodeHandle& nh) : nh_(nh), it_(nh_), tf_listener_(tf_buffer_), camera_info_received_(false), pan_camera_info_received_(false)
    {
        // 初始化图像发布者
        image_pub_ = it_.advertise("projected_image", 1);
        pan_image_pub_ = it_.advertise("projected_pan_image", 1);

        // 初始化订阅者
        rgb_sub_.subscribe(nh_, "/zed2/zed_node/rgb/image_rect_color/compressed", 1);
        equi_rgb_sub_.subscribe(nh_, "/camera/color/image_raw/compressed", 1);
        cloud_sub_.subscribe(nh_, "/ouster/points", 1);

        // 订阅相机内参
        camera_info_sub_ = nh_.subscribe("/zed2/zed_node/rgb/camera_info", 1, &SyncSensors::cameraInfoCallback, this);
        pan_camera_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &SyncSensors::panCameraInfoCallback, this);

        // 初始化同步器
        sync_.reset(new Sync(MySyncPolicy(10), rgb_sub_, equi_rgb_sub_, cloud_sub_));
        sync_->registerCallback(boost::bind(&SyncSensors::callback, this, _1, _2, _3));
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
    {
        if (!camera_info_received_)
        {
            fx_ = camera_info_msg->K[0];
            fy_ = camera_info_msg->K[4];
            cx_ = camera_info_msg->K[2];
            cy_ = camera_info_msg->K[5];

            camera_frame_ = camera_info_msg->header.frame_id;

            camera_info_received_ = true;

            ROS_INFO("Camera intrinsics received: fx=%f, fy=%f, cx=%f, cy=%f", fx_, fy_, cx_, cy_);
            ROS_INFO("Camera frame: %s", camera_frame_.c_str());
        }
    }

    void panCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
    {
        if (!pan_camera_info_received_)
        {
            fx_ = camera_info_msg->K[0];
            fy_ = camera_info_msg->K[4];
            cx_ = camera_info_msg->K[2];
            cy_ = camera_info_msg->K[5];

            image_width = camera_info_msg->width;
            image_height = camera_info_msg->height;

            pan_camera_frame_ = camera_info_msg->header.frame_id;

            pan_camera_info_received_ = true;

            ROS_INFO("Panoptic Camera intrinsics received: fx=%f, fy=%f, cx=%f, cy=%f", fx_, fy_, cx_, cy_);
            ROS_INFO("Panoptic Camera size received: width=%d, height=%d", image_width, image_height);
            ROS_INFO("Panoptic Camera frame: %s", pan_camera_frame_.c_str());
        }
    }

    std::pair<int, int> projectPointToEquirectangular(double X, double Y, double Z) {
        // 计算方位角 theta 和俯仰角 phi
        double theta = atan2(X, Z); // 水平角度
        double phi = asin(-Y / sqrt(X * X + Y * Y + Z * Z)); // 垂直角度

        // 将角度映射到图像坐标
        int u = static_cast<int>(image_width * ((theta + M_PI) / (2 * M_PI)));
        int v = static_cast<int>(image_height * ((M_PI/2 - phi) / M_PI));

        // 限制坐标范围，以确保投影结果在图像范围内
        u = std::max(0, std::min(u, image_width - 1));
        v = std::max(0, std::min(v, image_height - 1));

        return std::make_pair(u, v);
    }

    void callback(const sensor_msgs::CompressedImageConstPtr& rgb_msg,
                  const sensor_msgs::CompressedImageConstPtr& panoptic_msg,
                  const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        if (!camera_info_received_)
        {
            ROS_WARN("Waiting for camera intrinsics...");
            return;
        }

        try {
            // 解压缩RGB图像
            cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr cv_panoptic_rgb = cv_bridge::toCvCopy(panoptic_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat rgb_image = cv_rgb->image;
            cv::Mat panoptic_image = cv_panoptic_rgb->image;

            // 将点云消息转换为PCL点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);

            // 获取激光雷达到ZED相机的变换
            geometry_msgs::TransformStamped transform_stamped;
            try
            {
                // 使用相机的frame_id作为目标坐标系
                transform_stamped = tf_buffer_.lookupTransform(camera_frame_, cloud_msg->header.frame_id,
                                                               cloud_msg->header.stamp, ros::Duration(0.1));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Could not get transform: %s", ex.what());
                return;
            }
            // 获取激光雷达到ZED相机的变换
            geometry_msgs::TransformStamped transform_panoptic_stamped;
            try
            {
                // 使用相机的frame_id作为目标坐标系
                transform_panoptic_stamped = tf_buffer_.lookupTransform(pan_camera_frame_, cloud_msg->header.frame_id,
                                                               cloud_msg->header.stamp, ros::Duration(0.1));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Could not get transform: %s", ex.what());
                return;
            }

            // 将点云转换到ZED相机坐标系
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam(new pcl::PointCloud<pcl::PointXYZ>);
            pcl_ros::transformPointCloud(*cloud, *cloud_cam, transform_stamped.transform);
            // 将点云转换到全景相机坐标系
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pan_cam(new pcl::PointCloud<pcl::PointXYZ>);
            pcl_ros::transformPointCloud(*cloud, *cloud_pan_cam, transform_panoptic_stamped.transform);

            // 创建一个用于投影的图像
            cv::Mat projected_image = rgb_image.clone();
            cv::Mat projected_pan_image = panoptic_image.clone();

            // 投影点云到ZED图像平面
            for (const auto& point : cloud_cam->points)
            {
                if (point.z <= 0)
                    continue;

                // 投影到图像平面
                double u = fx_ * point.x / point.z + cx_;
                double v = fy_ * point.y / point.z + cy_;

                // 检查像素是否在图像范围内
                if (u >= 0 && u < projected_image.cols && v >= 0 && v < projected_image.rows)
                {
                    double D = point.z; // 点到相机的距离
                    double min_D = 0.0;
                    double max_D = 50.0; // 最大距离，需根据实际情况调整

                    // 计算颜色强度，距离越近颜色越深（即强度越低）
                    double intensity = 255.0 * (1.0 - (D - min_D) / (max_D - min_D));
                    intensity = std::max(0.0, std::min(255.0, intensity));

                    // 在图像上绘制点
                    cv::circle(projected_image, cv::Point(u, v), 1, cv::Scalar(intensity, intensity, intensity), -1);
                }
            }

            // 投影点云到全景图像平面
            for (const auto& point : cloud_pan_cam->points)
            {
                // if (point.z <= 0)
                //     continue;
                // 投影到图像平面
                std::pair<int, int> uv = projectPointToEquirectangular(point.x, point.y, point.z);
                int u = uv.first;
                int v = uv.second;

                // 检查像素是否在图像范围内
                if (u >= 0 && u < projected_pan_image.cols && v >= 0 && v < projected_pan_image.rows)
                {
                    double D = abs(point.z); // 点到相机的距离
                    double min_D = 0.0;
                    double max_D = 50.0; // 最大距离，需根据实际情况调整

                    // 计算颜色强度，距离越近颜色越深（即强度越低）
                    double intensity = 255.0 * (1.0 - (D - min_D) / (max_D - min_D));
                    intensity = std::max(0.0, std::min(255.0, intensity));

                    // 在图像上绘制点
                    cv::circle(projected_pan_image, cv::Point(u, v), 1, cv::Scalar(intensity,intensity,intensity), -1);
                }
            }

            // 发布投影后的图像
            sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(rgb_msg->header, "bgr8", projected_image).toImageMsg();
            image_pub_.publish(output_msg);
            // 发布投影后的图像
            sensor_msgs::ImagePtr pan_output_msg = cv_bridge::CvImage(panoptic_msg->header, "bgr8", projected_pan_image).toImageMsg();
            pan_image_pub_.publish(pan_output_msg);

            ROS_INFO("Projected image published.");

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        } catch (pcl::IOException& e) {
            ROS_ERROR("pcl exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher pan_image_pub_;

    // 相机内参
    double fx_, fy_, cx_, cy_;
    bool camera_info_received_;
    double p_fx_, p_fy_, p_cx_, p_cy_;
    bool pan_camera_info_received_;
    std::string camera_frame_;
    std::string pan_camera_frame_;
    int image_width;   // 图像宽度（像素数）
    int image_height;   // 图像高度（像素数）

    // TF变换
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 消息过滤器订阅者
    message_filters::Subscriber<sensor_msgs::CompressedImage> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> equi_rgb_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;

    // 相机信息订阅者
    ros::Subscriber camera_info_sub_;
    ros::Subscriber pan_camera_info_sub_;

    // 同步器
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_sensors");
    ros::NodeHandle nh;

    // 创建同步对象
    SyncSensors sync_sensors(nh);

    ROS_INFO("CALIBRATION TEST.");
    ros::spin();

    return 0;
}
