#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class TestSubscriber : public rclcpp::Node
{
public:
    TestSubscriber() : Node("test_subscriber_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image",
            10,
            [this](sensor_msgs::msg::Image::UniquePtr msg) {
                // convert the ROS image message to cv::Mat.
                cv::Mat frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char *>(msg->data.data()), msg->step);
                // Check if the frame is empty
                ASSERT_FALSE(frame.empty());
                // Check the frame properties
                ASSERT_EQ(frame.rows, msg->height);
                ASSERT_EQ(frame.cols, msg->width);
            });
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

TEST(CameraComponentTest, testImageReceived)
{
    auto test_subscriber = std::make_shared<TestSubscriber>();

    rclcpp::spin_some(test_subscriber);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    auto ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
