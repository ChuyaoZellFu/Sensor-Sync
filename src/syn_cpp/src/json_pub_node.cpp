#include <ros/ros.h>
#include "syn_cpp/BboxData.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>

using json = nlohmann::json;

int main(int argc, char** argv) {
    ros::init(argc, argv, "json_bbox_publisher");
    ros::NodeHandle nh;
    json data_j;
    // 初始化 BboxData 消息
    syn_cpp::BboxData bbox_msg;

    // 读取 JSON 文件
    std::string json_path;
    nh.param<std::string>("json_path", json_path, "2024-12-24-18-37-36.json");
    std::ifstream file(json_path);
    try {
        data_j = json::parse(file);
    } catch (const json::parse_error& e) {
        ROS_ERROR("JSON 解析失败: %s", e.what());
        return 0;
    }

    // 创建发布器
    ros::Publisher bbox_pub = nh.advertise<syn_cpp::BboxData>("bbox_data", 10);

    ros::Rate rate(100); // 控制播放速率

    for (auto& [ts_str, entry] : data_j.items()) {
        if (!ros::ok()) break;
        // --- 填充 header ---
        // --- 解析时间戳 ---
        try {
            // 将字符串类型的时间戳转换为 int64_t
            int64_t timestamp_ns = std::stoll(ts_str);
            bbox_msg.header.stamp = ros::Time().fromNSec(timestamp_ns); // 转换为 ROS 时间
        } catch (const std::exception& e) {
            ROS_ERROR("时间戳解析失败: %s (ts_str=%s)", e.what(), ts_str.c_str());
            continue;
        }
        // 根据实际设置坐标系
        bbox_msg.header.frame_id = "camera"; 

        // --- 填充 target_info（6 个元素）---
        if (entry.contains("target_info") && entry["target_info"].size() == 6) {
            for (const auto& val : entry["target_info"]) {
                bbox_msg.target_info.push_back(val.get<float>());
            }
        }

        // // --- 填充 det_bboxes（每 5 个元素一个框）---
        // if (entry.contains("det_bboxes")) {
        //     for (const auto& bbox : entry["det_bboxes"]) {
        //         // 格式: [x_min, y_min, x_max, y_max, score]
        //         for (size_t i = 0; i < 5; ++i) {
        //             bbox_msg.det_bboxes.push_back(bbox[i].get<float>());
        //             }
        //     }
        // }

        // // --- 填充 tracks_bbox（每 4 个元素一个框）---
        // if (entry.contains("tracks_target_conf_bbox")) {
        //     for (const auto& [track_id, track_data] : entry["tracks_target_conf_bbox"].items()) {
        //         // 假设 track_data[2] 是坐标数组 [x_min, y_min, x_max, y_max]
        //         const auto& coords = track_data[2];
        //         for (size_t i = 0; i < 4; ++i) {
        //             bbox_msg.tracks_bbox.push_back(coords[i].get<float>());
        //         }
        //     }
        // }

        // 发布消息
        bbox_pub.publish(bbox_msg);
        rate.sleep();
    }

    return 0;
}

