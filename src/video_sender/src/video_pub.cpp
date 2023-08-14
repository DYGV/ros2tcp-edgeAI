/*
 * Copyright 2023 Eisuke Okazaki
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <chrono>
#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define JPEG_QUALITY 80

using namespace std::chrono_literals;
typedef std::chrono::duration<float, std::milli> duration;
class VideoPublisher : public rclcpp::Node {
  public:
    VideoPublisher() : Node("video_publisher"), count(0) {
        declare_parameter("video_file", "~/video.mp4");
        declare_parameter("hz", 10);

        std::string video_file = get_parameter("video_file").as_string();
        int hz = get_parameter("hz").as_int();
        duration exec_interval{1000 / hz};
        publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "image", 10);
        timer = this->create_wall_timer(
            exec_interval, std::bind(&VideoPublisher::callback, this));

        cap.open(video_file);
        cap.set(cv::CAP_PROP_FPS, hz);

        param.push_back(cv::IMWRITE_JPEG_QUALITY);
        param.push_back(JPEG_QUALITY);
    }

    ~VideoPublisher() {
        cap.release();
        std::cout << "Video capture closed" << std::endl;
    }

  private:
    void callback() {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            return;
        }
        std::vector<uchar> buff;
        imencode(".jpg", frame, buff, param);
        auto msg = sensor_msgs::msg::CompressedImage();
        msg.data = buff;
        msg.format = "jpeg";
        publisher->publish(msg);
        std::cout << "Image Published: " << ++count << std::endl;
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
    cv::VideoCapture cap;
    unsigned long count;
    std::vector<int> param;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
