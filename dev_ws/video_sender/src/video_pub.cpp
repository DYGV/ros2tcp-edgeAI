#include <chrono>
#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
typedef std::chrono::duration<float, std::milli> duration;
class VideoPublisher : public rclcpp::Node {
  public:
    VideoPublisher() : Node("video_publisher"), count(0){
	declare_parameter("video_file", "~/video.mp4");
	declare_parameter("hz", 10);

	std::string video_file = get_parameter("video_file").as_string();
	int hz = get_parameter("hz").as_int();
	duration exec_interval {1000/hz};
        publisher =
            this->create_publisher<sensor_msgs::msg::Image>("image", 10);
        timer = this->create_wall_timer(
            exec_interval, std::bind(&VideoPublisher::callback, this));

        cap.open(video_file);
        cap.set(cv::CAP_PROP_FPS, hz);
    }

    ~VideoPublisher() {
        cap.release();
        std::cout << "Video capture closed" << std::endl;
    }

  private:
    void callback() {
        cv::Mat frame;
        cap >> frame;
        cv_bridge::CvImagePtr cv_ptr;
        if (frame.empty()) {
            return;
        }
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                .toImageMsg();
        publisher->publish(*msg.get());
        std::cout << "Image Published: " << ++count << std::endl;
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    cv::VideoCapture cap;
    unsigned long count;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
