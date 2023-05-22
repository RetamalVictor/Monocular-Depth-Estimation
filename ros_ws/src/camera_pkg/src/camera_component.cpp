#include "camera_pkg/camera_component.hpp"

namespace camera_composition
{
    // get mat encoding first
    std::string CameraComponent::mat_type2encoding(int mat_type)
    {
        switch (mat_type) {
            case CV_8UC1:
            return "mono8";
            case CV_8UC3:
            return "bgr8";
            case CV_16SC1:
            return "mono16";
            case CV_8UC4:
            return "rgba8";
            default:
            throw std::runtime_error("Unsupported encoding type");
        }
    }

    void CameraComponent::callback()
    {
        while (rclcpp::ok())
        {
            cap_m >> frame_m;
            if (frame_m.empty()){
                continue;
            }

            sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());

            // Pack the OpenCV image into the ROS image.
            msg->header.frame_id = "camera_frame";
            msg->height = frame_m.rows;
            msg->width = frame_m.cols;
            msg->encoding = mat_type2encoding(frame_m.type());
            msg->is_bigendian = false;
            msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_m.step);
            msg->data.assign(frame_m.datastart, frame_m.dataend);
            
            publisher_m->publish(std::move(msg));

        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_composition::CameraComponent)