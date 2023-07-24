#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <team_interfaces/msg/signs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Set up the path to the model
const std::string model_path = ament_index_cpp::get_package_share_directory("sign_detection") + "/model_files/best.onnx";

const std::vector<std::string> classes = {"cross_parking", "overtaking_allowed", "overtaking_forbidden",
                                          "parallel_parking", "pit_in", "pit_out"};

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor() : Node("image_processor") {
        // Load the model
        model_ = cv::dnn::readNetFromONNX(model_path);

        // Subscribe to the compressed image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/zed/zed_node/left/image_rect_color/compressed",
            10,
            std::bind(&ImageProcessor::processImage, this, std::placeholders::_1));

        // Create the publisher for detected signs
        publisher_ = this->create_publisher<team_interfaces::msg::Signs>("detected_signs", 10);

        cross_parking_ = false;
        pit_in_ = false;
        pit_out_ = false;
        overtaking_ = false;
    }

private:
    void processImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        // Convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Preprocess input
        int height = cvImage->image.rows;
        int width = cvImage->image.cols;
        int length = std::max(height, width);
        cv::Mat image = cv::Mat::zeros(length, length, CV_8UC3);
        cvImage->image.copyTo(image(cv::Rect(0, 0, width, height)));
        cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 255, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);

        // Set input to the model
        model_.setInput(blob);

        // Run inference
        cv::Mat outputs = model_.forward();

        // Preprocess output
        outputs = outputs.reshape(1, outputs.total() / 6);

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> class_ids;

        for (int i = 0; i < outputs.rows; ++i) {
            cv::Mat scores_class = outputs.row(i).colRange(4, outputs.cols);
            double min_score, max_score;
            cv::Point min_loc, max_loc;
            cv::minMaxLoc(scores_class, &min_score, &max_score, &min_loc, &max_loc);

            if (max_score >= 0.25) {
                cv::Rect box(outputs.at<float>(i, 0) - (0.5 * outputs.at<float>(i, 2)),
                             outputs.at<float>(i, 1) - (0.5 * outputs.at<float>(i, 3)),
                             outputs.at<float>(i, 2), outputs.at<float>(i, 3));
                boxes.push_back(box);
                scores.push_back(static_cast<float>(max_score));
                class_ids.push_back(max_loc.x);
            }
        }

        // Publish the detections
        auto result_msg = std::make_shared<team_interfaces::msg::Signs>();
        int max_class_id = 6;

        if (!scores.empty()) {
            int max_score_index = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
            max_class_id = class_ids[max_score_index];
            result_msg->sign_detected = true;
        } else {
            result_msg->sign_detected = false;
        }

        if (max_class_id == 0) {
            cross_parking_ = true;
        } else if (max_class_id == 1) {
            overtaking_ = true;
        } else if (max_class_id == 2) {
            overtaking_ = false;
        } else if (max_class_id == 3) {
            cross_parking_ = false;
        } else if (max_class_id == 4) {
            pit_in_ = true;
            pit_out_ = false;
        } else if (max_class_id == 5) {
            pit_out_ = true;
            pit_in_ = false;
        }

        result_msg->cross_parking = cross_parking_;
        result_msg->overtaking = overtaking_;
        result_msg->pit_in = pit_in_;
        result_msg->pit_out = pit_out_;

        publisher_->publish(result_msg);
    }

    cv::dnn::Net model_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<team_interfaces::msg::Signs>::SharedPtr publisher_;

    bool cross_parking_;
    bool pit_in_;
    bool pit_out_;
    bool overtaking_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto image_processor = std::make_shared<ImageProcessor>();
    rclcpp::spin(image_processor);
    rclcpp::shutdown();
    return 0;
}