import cv2
import endoscopy
import rclpy
from cv_bridge import CvBridge
from endoscopy.utils import yt_alpha_blend
from kornia.geometry import warp_perspective
from kornia.utils import image_to_tensor, tensor_to_image
from rclpy.node import Node
from sensor_msgs.msg import Image


class Inference(Node):
    def __init__(self):
        super().__init__("inference")
        self._cv_bridge = CvBridge()
        self._img_sub = self.create_subscription(Image, "image_rect", self._on_img, 1)
        self._h_est = endoscopy.HomographyEstimator(
            model=endoscopy.MODEL.HOMOGRAPHY_ESTIMATION.H_48_RESNET_34, device="cuda"
        )

        self._prev_torch_img = None

    def _on_img(self, msg: Image):
        cv_img = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        torch_img = image_to_tensor(cv_img, keepdim=False).float() / 255.0
        if self._prev_torch_img is None:
            self._prev_torch_img = torch_img
            return

        # estimate homography
        h, duv = self._h_est(torch_img, self._prev_torch_img)
        prev_torch_img_wrp = warp_perspective(
            self._prev_torch_img, h, torch_img.shape[-2:]
        )

        # blend warped previous image and current image
        blend = yt_alpha_blend(prev_torch_img_wrp, torch_img)
        cv_blend = (tensor_to_image(blend, keepdim=False) * 255).astype("uint8")

        # display results
        cv2.imshow("blend", cv_blend)
        cv2.waitKey(1)

        # save current image
        self._prev_torch_img = torch_img


def main():
    rclpy.init()
    inference = Inference()
    rclpy.spin(inference)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
