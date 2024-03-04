#include "bytetrack_viewer/bytetrack_viewer.hpp"

namespace bytetrack_viewer{
    cv::Scalar getColor(int id){
        int idx = id + 3;
        return cv::Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
    }
    void drawObject(cv::Mat frame, vision_msgs::msg::Detection2D box){
        // draw bbox
        auto color = getColor(static_cast<int>(std::stoi(box.id)));
        double bbox_xmin = box.bbox.center.position.x - (box.bbox.size_x / 2);
        double bbox_ymin = box.bbox.center.position.y - (box.bbox.size_y / 2);
        cv::rectangle(frame, 
                    cv::Rect( bbox_xmin, bbox_ymin, box.bbox.size_x, box.bbox.size_y),
                    color, 2);

        // draw ID
        float brightness = color[2] * 0.3 + color[1] * 0.59 + color[0] * 0.11;
        cv::Scalar txt_color;
        if (brightness > 127){
            txt_color = cv::Scalar(0, 0, 0);
        }else{
            txt_color = cv::Scalar(255, 255, 255);
        }

        std::string txt = cv::format("ID:%s %s", box.id.c_str(), box.results[0].hypothesis.class_id.c_str());
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseLine);
        cv::rectangle(frame, 
                      cv::Rect(cv::Point(bbox_xmin, bbox_ymin - label_size.height), 
                               cv::Size(label_size.width, label_size.height + baseLine)),
                      color, -1);
        cv::putText(frame, txt,
                    cv::Point(bbox_xmin, bbox_ymin), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, txt_color, 1, cv::LINE_AA);
    }

    ByteTrackViewer::ByteTrackViewer(const std::string &node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("bytetrack_viewer", node_name, options)
    {
    }
    ByteTrackViewer::ByteTrackViewer(const rclcpp::NodeOptions& options)
    : ByteTrackViewer("", options)
    {
        this->initializeParameter_();

        if (this->use_exact_sync_) {
            this->exact_sync_ = std::make_shared<ExactSynchronizer>(
                ExactSyncPolicy(this->queue_size_),
                sub_image_,
                sub_bboxes_);
            this->exact_sync_->registerCallback(
                std::bind(
                    &ByteTrackViewer::imageCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        } else {
            this->sync_ = std::make_shared<Synchronizer>(
                SyncPolicy(this->queue_size_), sub_image_, sub_bboxes_);
            this->sync_->registerCallback(
                std::bind(
                    &ByteTrackViewer::imageCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        }

        connectCallback();
        cv::namedWindow("ByteTrackViewer", cv::WINDOW_AUTOSIZE);

    }
    ByteTrackViewer::~ByteTrackViewer(){
        if(this->video_.isOpened()){
            this->video_.release();
            //RCLCPP_INFO(this->get_logger(), "save as " + this->save_video_name_ + ".");
        }
    }
    void ByteTrackViewer::initializeParameter_()
    {
        this->queue_size_ = this->declare_parameter<int>("queue_size", 5);
        this->use_exact_sync_ = this->declare_parameter<bool>("exact_sync", false);
        this->sub_image_topic_name_ = this->declare_parameter<std::string>("sub_image_topic_name", "/image_raw");
        this->sub_bboxes_topic_name_ = this->declare_parameter<std::string>("sub_bboxes_topic_name", "/bytetrack/bounding_boxes");
        this->save_video_ = this->declare_parameter<bool>("save_video", false);
        this->save_video_fps_ = this->declare_parameter<int>("save_video_fps", 30);
        this->save_video_name_ = this->declare_parameter<std::string>("save_video_name", "output.avi");
        this->save_video_codec_ = this->declare_parameter<std::string>("save_video_codec", "MJPG");
    }
    void ByteTrackViewer::connectCallback()
    {
        image_transport::TransportHints hints(this, "raw");
        this->sub_image_.subscribe(this, this->sub_image_topic_name_, hints.getTransport());
        this->sub_bboxes_.subscribe(this, this->sub_bboxes_topic_name_);
    }
    void ByteTrackViewer::imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr & trackers_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image and bboxes.");

        auto img = cv_bridge::toCvCopy(image_msg, "bgr8");
        cv::Mat frame = img->image;

        if (this->save_video_ && ! this->video_.isOpened()){
            this->video_ = cv::VideoWriter(
                this->save_video_name_,
                cv::VideoWriter::fourcc(
                    this->save_video_codec_.c_str()[0],
                    this->save_video_codec_.c_str()[1],
                    this->save_video_codec_.c_str()[2],
                    this->save_video_codec_.c_str()[3]),
                this->save_video_fps_,
                frame.size(),
                true);
        }

        auto bboxes = trackers_msg->detections;
        for(auto bbox: bboxes){
            drawObject(frame, bbox);
        }
        if (this->save_video_){
            this->video_ << frame;
        }
        cv::imshow("ByteTrackViewer", frame);
        auto key = cv::waitKey(1);
        if(key == 27){
            rclcpp::shutdown();
        }
    }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<bytetrack_viewer::ByteTrackViewer>(node_options));
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bytetrack_viewer::ByteTrackViewer)
