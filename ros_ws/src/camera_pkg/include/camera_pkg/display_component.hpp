#ifndef CAMERA_PKG__DISPLAY_HPP_
#define CAMERA_PKG__DISPLAY_HPP_

#include "visibility_control.h"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

namespace camera_composition
{
    class DisplayComponent final : public rclcpp::Node
    {
        public:
            explicit DisplayComponent(const rclcpp::NodeOptions& options) : Node("display", options)
            {
                std::string topic_name = "camera/depth_map";
                subscription_m = this->create_subscription<sensor_msgs::msg::Image>(topic_name, 10, std::bind(&DisplayComponent::callback, this, _1));
            }

            void callback(const sensor_msgs::msg::Image::UniquePtr msg);
            int encoding2mat_type(const std::string& encoding);
     
        private:
        cv::Mat frame_m;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_m;
    };
}
#endif