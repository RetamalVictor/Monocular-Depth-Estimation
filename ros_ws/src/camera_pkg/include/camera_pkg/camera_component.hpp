#ifndef CAMERA_PKG__CAMERA_NODE_HPP_
#define CAMERA_PKG__CAMERA_NODE_HPP_


#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <memory>

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
        explicit CameraComponent(const rclcpp::NodeOptions& options) : Node("camera", options)
        {
            std::string topic_name = "camera/image_raw";
            try
            {
                cap_m.open(2);
                if (!cap_m.isOpened()){
                    throw std::runtime_error("Failed to open camera");
                }
            } catch (const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Error while opening camera: %s", e.what());
                throw;
            }

            publisher_m = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
            thread_m = std::thread(std::bind(&CameraComponent::callback, this));        
        }

        ~CameraComponent()
        {
            if (thread_m.joinable()){
                thread_m.join();
            }
        }
    
        void callback();
        std::string mat_type2encoding(int mat_type);

    private:
    cv::VideoCapture cap_m;
    cv::Mat frame_m;
    std::thread thread_m;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_m;
    };
}
#endif