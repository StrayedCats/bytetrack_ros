#include "bytetrack_cpp_node/bytetrack_cpp_node.hpp"
#include "bytetrack_cpp_node/coco_names.hpp"

namespace bytetrack_cpp_node{
    using namespace bytetrack_cpp;

    std::vector<Object> Detection2Ds2Objects(const vector<vision_msgs::msg::Detection2D> detections)
    {
        std::vector<Object> objects;
        float scale = 1.0;
        for(auto detect: detections){
            for(auto result: detect.results){
                Object obj;
                obj.rect.x = (detect.bbox.center.position.x - (detect.bbox.size_x / 2)) / scale;
                obj.rect.y = (detect.bbox.center.position.y - (detect.bbox.size_y / 2)) / scale;
                obj.rect.width = (detect.bbox.size_x) / scale;
                obj.rect.height = (detect.bbox.size_y) / scale;

                auto it = std::find(COCO_CLASSES, COCO_CLASSES + 80, result.hypothesis.class_id);
                if (it != COCO_CLASSES + 80){
                    int idx = std::distance(COCO_CLASSES, it);
                    obj.label = idx;
                }

                obj.prob = result.hypothesis.score;
                objects.push_back(obj);
            }
        }
        return objects;
    }
    std::vector<vision_msgs::msg::Detection2D> STrack2Detection2Ds(const std::vector<STrack> trackers)
    {
        std::vector<vision_msgs::msg::Detection2D> detections;
        for(int i=0; i<trackers.size(); i++){
            vision_msgs::msg::Detection2D detection;
            detection.bbox.center.position.x = trackers[i].tlbr[1] + trackers[i].tlbr[3] / 2;
            detection.bbox.center.position.y = trackers[i].tlbr[0] + trackers[i].tlbr[2] / 2;
            detection.bbox.size_x = trackers[i].tlbr[3] - trackers[i].tlbr[1];
            detection.bbox.size_y = trackers[i].tlbr[2] - trackers[i].tlbr[0];
            detection.id = trackers[i].track_id;
        
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = COCO_CLASSES[trackers[i].label];
            hypothesis.hypothesis.score = trackers[i].score;

            detection.results.push_back(hypothesis);
            // bbox.center_dist = 0.0;
            // bbox.img_height = 0;
            // bbox.img_width = 0;
            detections.push_back(detection);
        }
        return detections;
    }
    
    ByteTrackNode::ByteTrackNode(const std::string &node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("bytetrack_cpp_node", node_name, options)
    {
    }
    ByteTrackNode::ByteTrackNode(const rclcpp::NodeOptions& options)
    : ByteTrackNode("", options)
    {
        this->initializeParameter_();
        this->tracker_ = std::make_unique<BYTETracker>(this->video_fps_, this->track_buffer_);

        this->sub_bboxes_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
                            this->sub_bboxes_topic_name_, 10, 
                            std::bind(&ByteTrackNode::topic_callback_, 
                                      this,
                                      std::placeholders::_1));
        this->pub_bboxes_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            this->pub_bboxes_topic_name_,
            10
        );
    }
    void ByteTrackNode::initializeParameter_()
    {
        this->video_fps_ = this->declare_parameter<int>("video_fps", 30);
        this->track_buffer_ = this->declare_parameter<int>("track_buffer", 30);
        this->sub_bboxes_topic_name_ = this->declare_parameter<std::string>("sub_bboxes_topic_name", "yolox/bounding_boxes");
        this->pub_bboxes_topic_name_ = this->declare_parameter<std::string>("pub_bboxes_topic_name", "bytetrack/bounding_boxes");
    }
    void ByteTrackNode::topic_callback_(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
    {
        vision_msgs::msg::Detection2DArray boxes;
        boxes.header = msg->header;
    
        vector<Object> objects = Detection2Ds2Objects(msg->detections);
        vector<STrack> output_stracks = this->tracker_->update(objects);
        RCLCPP_INFO(this->get_logger(), "Detect objects: %d, Output Tracker: %d", objects.size(), output_stracks.size());
        boxes.detections = STrack2Detection2Ds(output_stracks);
        this->pub_bboxes_->publish(boxes);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<bytetrack_cpp_node::ByteTrackNode>(node_options));
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bytetrack_cpp_node::ByteTrackNode)