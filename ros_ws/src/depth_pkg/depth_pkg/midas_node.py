import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2
import numpy as np
import torch

from depth_pkg.transform_pipeline import create_transform


class MiDaSNode(Node):

    def __init__(self):
        super().__init__('midas_node')

        # Declare parameters
        self.declare_parameter('model_type', 'MiDaS_small')
        self.model_type = self.get_parameter('model_type').get_parameter_value().string_value

        self.get_logger().info(f'Loading MiDaS model: {self.model_type}')
        self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type)
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.get_logger().info(f'Using device: {self.device}')
        self.midas.to(self.device)
        self.midas.eval()
        self.transform = create_transform()

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.publisher_ = self.create_publisher(Image, 'camera/depth_map', 10)

    def image_callback(self, msg):
        try:
            # Convert ROS2 Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            input_batch = self.transform(img).to(self.device)

            with torch.no_grad():
                prediction = self.midas(input_batch)

                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=img.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()

            output = prediction.cpu().numpy()

            # Convert the depth map to a single-channel grayscale image (CV_8U for mono8)
            depth_map_image = cv2.normalize(
                output, None, 0, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )

            # Convert the OpenCV image to a ROS Image message
            depth_map_msg = self.bridge.cv2_to_imgmsg(depth_map_image, encoding="mono8")
            depth_map_msg.header = msg.header

            # Publish the depth map
            self.publisher_.publish(depth_map_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)

    # Instantiate the MiDaSNode
    midas_node = MiDaSNode()

    # Pass the node instance to spin
    rclpy.spin(midas_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
