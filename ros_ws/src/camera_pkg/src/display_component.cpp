#include "camera_pkg/display_component.hpp"

namespace camera_composition
{
    int DisplayComponent::encoding2mat_type(const std::string & encoding)
    {
        if (encoding == "mono8") {
            return CV_8UC1;
        } else if (encoding == "bgr8") {
            return CV_8UC3;
        } else if (encoding == "mono16") {
            return CV_16SC1;
        } else if (encoding == "rgba8") {
            return CV_8UC4;
        }
        throw std::runtime_error("Unsupported mat type");
    }

    void DisplayComponent::callback(const sensor_msgs::msg::Image::UniquePtr msg)
    {
        cv::Mat frame(
            msg->height, msg->width,
            encoding2mat_type(msg->encoding),
            msg->data.data()
        );
        cv::imshow("window", frame);
        cv::waitKey(1);
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_composition::DisplayComponent)