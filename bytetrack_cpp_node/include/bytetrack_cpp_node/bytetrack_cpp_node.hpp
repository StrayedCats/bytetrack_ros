#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#include <chrono>
#include <vector>
#include <dirent.h>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "bboxes_ex_msgs/msg/bounding_box.hpp"
#include "bboxes_ex_msgs/msg/bounding_boxes.hpp"

#include "bytetrack_cpp/BYTETracker.h"

namespace bytetrack_cpp_node{
    using namespace bytetrack_cpp;

    class ByteTrackNode : public rclcpp::Node
    {
    public:
        ByteTrackNode(const std::string &node_name, const rclcpp::NodeOptions& options);
        ByteTrackNode(const rclcpp::NodeOptions& options);

    private:
        void initializeParameter_();
        void topic_callback_(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);
        
        int video_fps_ = 30;
        int track_buffer_ = 30;
        std::string sub_bboxes_topic_name_;
        std::string pub_bboxes_topic_name_;
        std::unique_ptr<BYTETracker> tracker_;

        rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_bboxes_;
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_bboxes_;
    };
}