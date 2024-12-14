#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "mxcarkit_uss_message/msg/uss_custom_message.hpp"
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <sl/Camera.hpp>

class UssAckermannNode : public rclcpp::Node
{
public:
    UssAckermannNode() : Node("uss_ackermann_node")
    {
    
    	// Initialize ZED camera
    	if (!initZED()){
    		RCLCPP_ERROR(this->get_logger(), "Failed to initialize ZED camera");
    		rclcpp::shutdown();
    	}
    	timer_= this->create_wall_timer(
    		std::chrono::milliseconds(100), std::bind(&UssAckermannNode::processImage, this));
    	
 	// Configure the QoS profile to match the publisher
    	/*auto qos = rclcpp::QoS(1);  // Depth of 1
    	qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    	qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    	qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    	qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    	*/
        // Subscriber to the /uss_sensor topic with custom message type
        /* uss_subscriber_ = this->create_subscription<mxcarkit_uss_message::msg::USSCustomMessage>(
            "/uss_sensors", qos,
            std::bind(&UssAckermannNode::uss_callback, this, std::placeholders::_1));
	*/
        // Publisher to the /ackermann_cmd topic
        /*ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/ackermann_cmd", 10);*/
    }
    ~UssAckermannNode(){
    	zed_.close();
    }

private:
    sl::Camera zed_;
    sl::Mat zed_image_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool initZED(){
    	sl::InitParameters init_params;
    	init_params.camera_resolution =  sl::RESOLUTION::HD720;
    	init_params.camera_fps = 30;
    	sl::ERROR_CODE err = zed_.open(init_params);
    	if (err !=sl::ERROR_CODE::SUCCESS){
    		RCLCPP_ERROR(this->get_logger(), "ERROR opening ZED camera : %s", sl::toString(err).c_str());
    		return false;
    	}
    	return true;
    }
    
    void processImage() {
    //Capture image from ZED camera
    if (zed_.grab() == sl::ERROR_CODE::SUCCESS) {
    	zed_.retrieveImage(zed_image_, sl::VIEW::LEFT);
    	
    	//convert ZED image to opencv format
    	cv::Mat frame = cv::Mat(zed_image_.getHeight(), zed_image_.getWidth(), CV_8UC4, zed_image_.getPtr<sl::uchar1>(sl::MEM::CPU));
    	cv::imwrite("frame.png",frame);
    	
    	// applying bird view
    	cv::Mat bird_view;
    	bird_view = UssAckermannNode::BirdViewTransform(frame);
    	cv::imwrite("bird_view.png",bird_view);
    	
    	//filter lines
    	cv::Mat filtered_img;
    	filtered_img = UssAckermannNode::filter_line(bird_view);
    	cv::imwrite("filtered_img.png",filtered_img);
    	
    	
    	
    	
    	//Process the image to detect lanes
    	cv::Mat gray, edges;
    	cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    	cv::Canny(gray, edges, 50, 150);
    	
    	//Hough transform to detect lanes
    	/*std::vector<cv::Vec4i> lines;
    	cv::HoughLinesP(edges, lines, 1, CV_PI/180, 50, 50 ,10);
    	
    	for (size_t i=0; i<lines.size();i++){
    		 cv::line(frame, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255,0,0), 3, 8);
    	}*/
    	
    	//Display the processed image
    	//cv::imshow("Lane detection", frame);
    	/*cv::imwrite("frame.png",frame);
    	cv::imwrite("bird_view.png",bird_view);
    	cv::imwrite("filtered_img.png",filtered_img);*/
    	cv::waitKey(1);
    } else{
    	RCLCPP_WARN(this->get_logger(), "Failed to grab image from ZED camera.");
    }
    }
    
    //Bird view Transform
    
    // Camera output params 
    float width = 1280;
    float height = 720;
    float scale = 1;
    
    //BEV params
    float top_roi = 0.6100; // 0.4395 # 0-1 (0 is top)
    float bottom_roi = top_roi + 0.1952; //0.6347 # 0-1 (0 is top)
    float width_use = 1; // 0-1 (For cropping vertically. Not used)
    float height_multiplier = 3.5; // How much to stretch vertically
    float skew_level = 0.82; //0.887 # 0-1 (1 is triangle, 0 is rectangle)
           
    cv::Mat BirdViewTransform(cv::Mat& img) {
    	// Define the source points (manually selected based on lane location)
    	std::vector<cv::Point2f> src_pts = {
    		cv::Point2f((1-width_use)*width*scale,0), //Bottom-left
    		cv::Point2f((1-width_use)*width*scale,(bottom_roi-top_roi)*height*scale), //Bottom-right
        	cv::Point2f(width_use*width*scale,(bottom_roi-top_roi)*height*scale), //Top-left
    		cv::Point2f(width_use*width*scale,0), //Top-right
    	};
	// Define the destination points (for a top-down view)
    	std::vector<cv::Point2f> dst_pts = {
    		cv::Point2f((1-width_use)*width*scale,0), //Bottom-left
    		cv::Point2f(width*skew_level*0.5*width_use*scale,(bottom_roi-top_roi)*height*height_multiplier*scale), //Bottom-right
        	cv::Point2f(width*(1-skew_level*0.5*width_use)*scale,(bottom_roi-top_roi)*height*height_multiplier*scale), //Top-left
    		cv::Point2f(width_use*width*scale,0), //Top-right
    	};
    	
    	cv::Mat M=cv::getPerspectiveTransform(src_pts, dst_pts);
    	cv::Mat bird_view;
      	cv::Mat cropped_img = img.rowRange(int (top_roi*height), int (bottom_roi*height)).colRange(0, int (width));
    	cv::Size cropped_img_size = cropped_img.size();
    	cv::warpPerspective(cropped_img, bird_view, M, cv::Size(cropped_img_size.width,cropped_img_size.height));
    	return bird_view;
    }
    
    // thresholding to get binary image
    int gaussian = 13; // must be odd number
    int thresh = 120; //94 # 0-255 (lower means higher sensitivity) 
    cv::Mat filter_line(cv::Mat& img){
    	cv::Mat img_hls;
    	cv::cvtColor(img, img_hls, cv::COLOR_BGR2HLS);
    	std::vector<cv::Mat> hls_channels;
    	cv::split(img_hls, hls_channels);
    	cv::Mat lightness_channel = hls_channels[1];
    	cv::GaussianBlur(lightness_channel, lightness_channel, cv::Size(gaussian, gaussian),0);
    	cv::Mat img_thresh;
    	cv::threshold(lightness_channel, img_thresh, thresh, 255, cv::THRESH_BINARY);
    	return img_thresh;
    }
    
    /*void uss_callback(const mxcarkit_uss_message::msg::USSCustomMessage::SharedPtr msg)
    {
        // Process the ultrasonic sensor data
        int16_t range = msg->uss_range[8];
        RCLCPP_INFO(this->get_logger(), "Received USS data: Range=%d", range);
	

	
        // Create an Ackermann message
        auto ackermann_msg = ackermann_msgs::msg::AckermannDrive();
	
        // Example logic: Adjust the speed and steering based on the distance
        if (range < 10.0) // If the obstacle is closer than 1 meter
        {
            ackermann_msg.speed = 0.0; // Stop
            ackermann_msg.steering_angle = 0.0; // Go straight
        }
        
        else if (range < 20.0)
        {
            ackermann_msg.speed = 0.0; // Move forward
            ackermann_msg.steering_angle = 0.0; // Go straight
        }
        else
        {
            ackermann_msg.speed = 0.0; // Move forward
            ackermann_msg.steering_angle = 0.0; // Go straight
        }

        // Publish the Ackermann command
        ackermann_publisher_->publish(ackermann_msg);
    }

    rclcpp::Subscription<mxcarkit_uss_message::msg::USSCustomMessage>::SharedPtr uss_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_publisher_;
    */
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UssAckermannNode>());
    rclcpp::shutdown();
    return 0;
}
