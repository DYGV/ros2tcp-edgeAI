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
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/bind/bind.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/json.hpp>
#include <boost/json/src.hpp>

#include "pose_estimation_msg/msg/point.hpp"
#include "pose_estimation_msg/msg/pose_estimation_result.hpp"
#include "pose_estimation_msg/msg/result.hpp"

using boost::asio::ip::tcp;

class PoseEstimationTcp : public rclcpp::Node {
  public:
    PoseEstimationTcp(boost::asio::io_service &io_service)
        : Node("pose_estimation_tcp"), socket(io_service) {

        declare_parameter("server_addr", "192.168.0.2");
        declare_parameter("server_port", 54321);
        declare_parameter("show_gui", false);

        std::string server_addr = get_parameter("server_addr").as_string();
        int server_port = get_parameter("server_port").as_int();
        show_gui = get_parameter("show_gui").as_bool();

        subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "image", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) { callback(msg); });
        result_publisher =
            this->create_publisher<pose_estimation_msg::msg::Result>(
                "pose_estimation_result", 10);

        tcp::resolver resolver(io_service);
        tcp::resolver::query query(server_addr, std::to_string(server_port));
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::connect(socket, endpoint_iterator);
        socket.set_option(tcp::no_delay(true));
    }

    ~PoseEstimationTcp() { socket.close(); }

  private:
    void send_frame(cv::Mat &frame) {
        if (frame.empty()) {
            return;
        }
        if (frame.cols != 368 || frame.rows != 368) {
            cv::resize(frame, frame, cv::Size(368, 368));
        }
        std::size_t frame_size = frame.total() * frame.elemSize();
        boost::asio::write(socket, boost::asio::buffer(frame.data, frame_size));
    }

    boost::json::value recv_result() {
        std::size_t result_size;
        boost::asio::read(
            socket, boost::asio::buffer(&result_size, sizeof(std::size_t)));
        std::vector<char> receive_buffer(result_size);
        boost::asio::read(
            socket, boost::asio::buffer(receive_buffer.data(), result_size));
        std::string receive_str(receive_buffer.begin(), receive_buffer.end());
        std::cout << receive_str << std::endl;
        return boost::json::parse(receive_str);
    }

    void test_imshow(cv::Mat &frame, pose_estimation_msg::msg::Result result) {
        std::vector<std::vector<int>> limb_seq = {
            {0, 1}, {1, 2}, {2, 3},  {3, 4},  {1, 5},   {5, 6},  {6, 7},
            {1, 8}, {8, 9}, {9, 10}, {1, 11}, {11, 12}, {12, 13}};
        for (size_t k = 1; k < result.poses.size(); ++k) {
            for (size_t i = 0; i < result.poses[k].pose.size(); ++i) {
                if (result.poses[k].pose[i].is_valid == 1) {
                    cv::Point2f point(result.poses[k].pose[i].x,
                                      result.poses[k].pose[i].y);
                    cv::circle(frame, point, 5, cv::Scalar(0, 255, 0), -1);
                }
            }
            for (size_t i = 0; i < limb_seq.size(); ++i) {
                auto p_0 = result.poses[k].pose[limb_seq[i][0]];
                auto p_1 = result.poses[k].pose[limb_seq[i][1]];
                if (p_0.is_valid == 1 && p_1.is_valid == 1) {
                    cv::line(frame, cv::Point2f(p_0.x, p_0.y),
                             cv::Point2f(p_1.x, p_1.y), cv::Scalar(255, 0, 0),
                             3, 4);
                }
            }
        }
        cv::imshow("result", frame);
        cv::waitKey(1);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, msg->encoding)->image;
        send_frame(frame);
        auto pub_msg = make_pub_msg(recv_result());
        result_publisher->publish(pub_msg);
        if (show_gui) {
            test_imshow(frame, pub_msg);
        }
    }

    pose_estimation_msg::msg::Result make_pub_msg(boost::json::value result) {
        auto msg = pose_estimation_msg::msg::Result();
        msg.height = result.at("height").to_number<int>();
        msg.width = result.at("width").to_number<int>();
        auto result_poses = result.at("poses");
        for (const auto &outer_pair : result_poses.as_object()) {
            const auto &outer_value = outer_pair.value();
            auto result_msg = pose_estimation_msg::msg::PoseEstimationResult();
            auto point_msg = pose_estimation_msg::msg::Point();
            for (const auto &middle_pair : outer_value.as_object()) {
                const auto &middle_value = middle_pair.value();
                point_msg.x = middle_value.at("x").to_number<int>();
                point_msg.y = middle_value.at("y").to_number<int>();
                point_msg.is_valid = middle_value.at("type").to_number<int>();
                result_msg.pose.push_back(point_msg);
            }
            msg.poses.push_back(result_msg);
        }
        return msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Publisher<pose_estimation_msg::msg::Result>::SharedPtr
        result_publisher;
    tcp::socket socket;
    bool show_gui;
};

int main(int argc, char *argv[]) {
    boost::asio::io_service io_service;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseEstimationTcp>(io_service));
    rclcpp::shutdown();
    return 0;
}
