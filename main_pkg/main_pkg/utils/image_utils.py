import os

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def save_binary_mask(mask: np.ndarray, output_dir: str, file_name: str = "mask"):
    """
    Saves a batch of binary masks to image files.

    Args:
        mask (np.ndarray): Binary masks of shape [width, height]
        output_dir (str): Directory to save the images.
        file_name (str): Optional filenames.
    """
    os.makedirs(output_dir, exist_ok=True)

    mask = (mask * 255).astype(np.uint8)
    # Save as PNG
    path = os.path.join(output_dir, f"{file_name}.png")
    cv2.imwrite(path, mask)

    print(f"Saved mask to: {output_dir}")

def save_binary_masks(masks: np.ndarray, output_dir: str, file_name: str = "mask"):
    """
    Saves a batch of binary masks to image files.

    Args:
        masks (np.ndarray): Binary masks of shape [n, width, height]
        output_dir (str): Directory to save the images.
        file_name (str): Optional filenames.
    """
    os.makedirs(output_dir, exist_ok=True)
    n = masks.shape[0]

    for i in range(n):
        # Get the i-th mask and scale to 0–255 for visibility
        mask = (masks[i] * 255).astype(np.uint8)

        # Save as PNG
        path = os.path.join(output_dir, f"{file_name}_{i:03}.png")
        cv2.imwrite(path, mask)

    print(f"Saved {n} masks to: {output_dir}")


def save_rgb_image(rgb_image: np.ndarray, output_dir: str, file_name: str = "rgb"):
    """
    Saves rgb image.

    Args:
        rgb_image (np.ndarray): RGB Image
        output_dir (str): Directory to save the images.
        file_name (str): Optional filenames.
    """
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{file_name}.png")
    cv2.imwrite(path, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))  # OpenCV uses BGR
    print(f"Saved RGB to: {path}")


def save_depth_uint16(depth_maps: np.ndarray, output_dir: str, file_name: str = "depth"):
    """
    Saves Depth image.

    Args:
        depth_maps (np.ndarray): Depth Image
        output_dir (str): Directory to save the images.
        file_name (str): Optional file_name for filenames.
    """
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{file_name}.png")

    # Save as 16-bit PNG (supports uint16)
    cv2.imwrite(path, depth_maps.astype(np.uint16))
    print(f"Saved Depth to: {path}")


def mask_to_ros_image(mask: np.ndarray) -> Image:
    bridge = CvBridge()

    # Ensure mask is uint8 (0 or 255 for visualization, or keep as 0/1)
    if mask.dtype != np.uint8:
        mask = mask.astype(np.uint8)

    # Optionally scale 0/1 to 0/255 for visibility
    mask = mask * 255

    # Convert to ROS Image (mono8)
    ros_image = bridge.cv2_to_imgmsg(mask, encoding="mono8")
    return ros_image


def rgb_to_ros_image(rgb: np.ndarray) -> Image:
    bridge = CvBridge()
    return bridge.cv2_to_imgmsg(rgb, encoding="8UC3")