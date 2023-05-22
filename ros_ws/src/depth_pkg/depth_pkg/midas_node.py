import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2
import torch

from depth_pkg.transform_pipeline import create_transform

class MiDaSNode(Node):

    def __init__(self):
        super().__init__('midas_node')
        
        self.model_type = "MiDaS_small"  
        self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type) 
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.midas.to(self.device)
        self.midas.eval()
        self.transform = create_transform()
        
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Image, 'camera/depth_map', 10)
        
    def mat_type2encoding(self, mat_type):
        encoding = {
            cv2.CV_8UC1: 'mono8',
            cv2.CV_8UC3: 'bgr8',
            cv2.CV_16SC1: 'mono16',
            cv2.CV_8UC4: 'rgba8',
        }

        if mat_type in encoding:
            return encoding[mat_type]
        else:
            raise ValueError('Unsupported encoding type')
    def encoding2mat_type(self, encoding):
        if encoding == 'mono8':
            return cv2.CV_8UC1
        elif encoding == 'bgr8':
            return cv2.CV_8UC3
        elif encoding == 'mono16':
            return cv2.CV_16SC1
        elif encoding == 'rgba8':
            return cv2.CV_8UC4
        else:
            raise RuntimeError('Unsupported encoding type')

    def image_callback(self, msg):
        # encoding = self.mat_type2encoding(msg.)
        # Convert ROS2 Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding) # need to generalize to other encodings
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(img).to(self.device)
        # print("transformed")
        
        with torch.no_grad():
            prediction = self.midas(input_batch)

            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        output = prediction.cpu().numpy()
        # # Convert the depth map to a grayscale image
        depth_map_image = cv2.normalize(output, None, 255, 0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC3)
        # depth_map_image = cv2.normalize(output, None, 255, 0, norm_type=cv2.NORM_MINMAX, dtype=self.encoding2mat_type(msg.encoding))
        # print(depth_map_image)
        
        # # Convert the OpenCV image to a ROS Image message
        # print(msg.encoding)
        depth_map_msg = self.bridge.cv2_to_imgmsg(depth_map_image, encoding="mono8")
        depth_map_msg.header = msg.header

        # Publish the depth map
        self.publisher_.publish(depth_map_msg)

def main(args=None):
    rclpy.init(args=args)

    # Instantiate the MiDaSNode
    midas_node = MiDaSNode()

    # Pass the node instance to spin
    rclpy.spin(midas_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
