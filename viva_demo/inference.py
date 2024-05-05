import endoscopy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class Inference(Node):
    def __init__(self):
        super().__init__("inference")
        self._cv_bridge = CvBridge()
        self._img_sub = self.create_subscription(Image, "image", self._on_img, 1)

    def _on_img(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("image", cv_image)
        cv2.waitKey(1)


def main():
    rclpy.init()
    inference = Inference()
    rclpy.spin(inference)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
