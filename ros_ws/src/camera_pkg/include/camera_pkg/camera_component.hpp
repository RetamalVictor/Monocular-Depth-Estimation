#ifndef CAMERA_PKG__CAMERA_NODE_HPP_
#define CAMERA_PKG__CAMERA_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace camera_composition
{
    class CameraComponent final : public rclcpp::Node
    {
        /*This class initializes the camera capture and the frame.*/

    public:
        explicit CameraComponent(const rclcpp::NodeOptions& options) : Node("camera", options), running_(true)
        {
            // Declare parameters
            this->declare_parameter<int>("camera_index", 0);
            this->declare_parameter<std::string>("topic_name", "camera/image_raw");
            this->declare_parameter<int>("frame_width", 640);
            this->declare_parameter<int>("frame_height", 480);

            int camera_index = this->get_parameter("camera_index").as_int();
            std::string topic_name = this->get_parameter("topic_name").as_string();
            int frame_width = this->get_parameter("frame_width").as_int();
            int frame_height = this->get_parameter("frame_height").as_int();

            RCLCPP_INFO(get_logger(), "Opening camera device: %d", camera_index);

            try
            {
                cap_m.open(camera_index);
                if (!cap_m.isOpened()) {
                    throw std::runtime_error("Failed to open camera device " + std::to_string(camera_index));
                }
                cap_m.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
                cap_m.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
                RCLCPP_INFO(get_logger(), "Camera opened successfully: %dx%d", frame_width, frame_height);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Error while opening camera: %s", e.what());
                throw;
            }

            publisher_m = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
            thread_m = std::thread(std::bind(&CameraComponent::callback, this));
        }

        ~CameraComponent()
        {
            running_ = false;
            if (thread_m.joinable()) {
                thread_m.join();
            }
            if (cap_m.isOpened()) {
                cap_m.release();
            }
        }

        void callback();
        std::string mat_type2encoding(int mat_type);

    private:
        cv::VideoCapture cap_m;
        cv::Mat frame_m;
        std::thread thread_m;
        std::atomic<bool> running_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_m;
    };
}
#endif