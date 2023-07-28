#include <chrono>
#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/bind/bind.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/json.hpp>
#include <boost/json/src.hpp>

#include "face_detection_msg/msg/face_detection_result.hpp"
#include "face_detection_msg/msg/result.hpp"

using boost::asio::ip::tcp;

class FaceDetectionTcp : public rclcpp::Node {
  public:
    FaceDetectionTcp(boost::asio::io_service &io_service)
        : Node("face_detection_tcp"), socket(io_service) {

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
            this->create_publisher<face_detection_msg::msg::Result>(
                "face_detection_result", 10);

        tcp::resolver resolver(io_service);
        tcp::resolver::query query(server_addr, std::to_string(server_port));
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::connect(socket, endpoint_iterator);
        socket.set_option(tcp::no_delay(true));
    }

    ~FaceDetectionTcp() { socket.close(); }

  private:
    void send_frame(cv::Mat &frame) {
        if (frame.empty()) {
            return;
        }
        if(frame.cols != 640 || frame.rows != 360) {
            cv::resize(frame, frame, cv::Size(640, 360));
        }
        std::size_t frame_size =
            frame.total() * frame.elemSize();
        boost::asio::write(socket,
                           boost::asio::buffer(frame.data, frame_size));
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

    void test_imshow(cv::Mat &frame, face_detection_msg::msg::Result result) {
        for (const auto &msg : result.face_detection_result) {
            cv::rectangle(
                frame,
                cv::Rect{cv::Point(msg.x, msg.y),
                         cv::Size{int(msg.size_col), int(msg.size_row)}},
                cv::Scalar(255, 0, 0), 3, 3);
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

    face_detection_msg::msg::Result make_pub_msg(boost::json::value result) {
        auto msg = face_detection_msg::msg::Result();
        int num = 0;
        auto it = result.as_object().find("num");
        if (it != result.as_object().end()) {
            num = it->value().to_number<int>();
        } else {
            return msg;
        }
        msg.height = result.at("height").to_number<int>();
        msg.width = result.at("width").to_number<int>();
        for (int i = 1; i <= num; i++) {
            auto result_msg = face_detection_msg::msg::FaceDetectionResult();
            auto result_pos = result.at(std::to_string(i));
            result_msg.x = result_pos.at("x").to_number<int>();
            result_msg.y = result_pos.at("y").to_number<int>();
            result_msg.size_col = result_pos.at("size_col").to_number<int>();
            result_msg.size_row = result_pos.at("size_row").to_number<int>();
            msg.face_detection_result.push_back(result_msg);
        }
        return msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Publisher<face_detection_msg::msg::Result>::SharedPtr
        result_publisher;
    tcp::socket socket;
    bool show_gui;
};

int main(int argc, char *argv[]) {
    boost::asio::io_service io_service;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceDetectionTcp>(io_service));
    rclcpp::shutdown();
    return 0;
}
