#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, PointCloud2
from cv_bridge import CvBridge
import cv2
import message_filters
import numpy as np
import os
import sensor_msgs.point_cloud2 as pc2
import pcl

global _dir, _zed_dir, _theta_dir, _ouster_dir

def callback(zed_img_msg, theta_img_msg, pointcloud_msg):
    global _dir, _zed_dir, _theta_dir, _ouster_dir
    bridge = CvBridge()

    try:
        # 解压缩RGB图像
        zed_img = bridge.compressed_imgmsg_to_cv2(zed_img_msg, "bgr8")
        # 解压缩深度图像
        theta_img = bridge.compressed_imgmsg_to_cv2(theta_img_msg, "bgr8")
        # 处理点云数据（这里只是一个示例，具体处理根据需要进行）
        pointcloud_data = pointcloud_msg.data

        # 获取当前时间戳
        zed_img_timestamp = zed_img_msg.header.stamp
        # _timestamp = rgb_msg.header.stamp
        theta_img_timestamp = theta_img_msg.header.stamp
        pcs_timestamp = pointcloud_msg.header.stamp

        # 保存图像和点云数据
        cv2.imwrite(os.path.join(_zed_dir, "{}.jpg".format(zed_img_timestamp)), zed_img)
        cv2.imwrite(os.path.join(_theta_dir, "{}.jpg".format(theta_img_timestamp)), theta_img)
        # 保存点云数据，这里假设你希望保存为numpy数组
        cloud_points = list(pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True))
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_list(cloud_points)
        pcl.save(pcl_cloud, os.path.join(_ouster_dir, "{}.pcd".format(pcs_timestamp)))

        rospy.loginfo("Images and PointCloud saved for timestamp: {}".format(theta_img_timestamp))

    except Exception as e:
        rospy.logerr("Error in callback: {}".format(e))

def main():
    global _dir, _zed_dir, _theta_dir, _ouster_dir
    rospy.init_node('sync_images', anonymous=True)

    zed_img_sub = message_filters.Subscriber('/zed2/zed_node/rgb/image_rect_color/compressed', CompressedImage)
    theta_img_sub = message_filters.Subscriber('/camera/color/image_raw/compressed', CompressedImage)
    pointcloud_sub = message_filters.Subscriber('/ouster/points', PointCloud2)

    _dir = "/home/hjyeee/Data/Dataset/rpf_dataset/2024-07-04-21-12-25"
    _zed_dir = os.path.join(_dir, "zed_images")
    _theta_dir = os.path.join(_dir, "theta_images")
    _ouster_dir = os.path.join(_dir, "ouster_pcs")
    if not os.path.exists(_zed_dir):
        os.mkdir(_zed_dir)
    if not os.path.exists(_theta_dir):
        os.mkdir(_theta_dir)
    if not os.path.exists(_ouster_dir):
        os.mkdir(_ouster_dir)

    ts = message_filters.ApproximateTimeSynchronizer([zed_img_sub, theta_img_sub, pointcloud_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback)
    rospy.loginfo("Time Synchronizer Registered")

    rospy.spin()

if __name__ == '__main__':
    main()