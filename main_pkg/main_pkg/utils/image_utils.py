import os

import cv2
import numpy as np


def save_binary_masks(masks: np.ndarray, output_dir: str, prefix: str = "mask"):
    """
    Saves a batch of binary masks to image files.

    Args:
        masks (np.ndarray): Binary masks of shape [n, width, height]
        output_dir (str): Directory to save the images.
        prefix (str): Optional prefix for filenames.
    """
    os.makedirs(output_dir, exist_ok=True)
    n = masks.shape[0]

    for i in range(n):
        # Get the i-th mask and scale to 0–255 for visibility
        mask = (masks[i] * 255).astype(np.uint8)

        # Save as PNG
        path = os.path.join(output_dir, f"{prefix}_{i:03}.png")
        cv2.imwrite(path, mask)

    print(f"Saved {n} masks to: {output_dir}")


def save_rgb_images(rgb_images: np.ndarray, output_dir: str, prefix: str = "rgb"):
    """
    Saves rgb image.

    Args:
        rgb_images (np.ndarray): RGB Image
        output_dir (str): Directory to save the images.
        prefix (str): Optional prefix for filenames.
    """
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{prefix}.png")
    cv2.imwrite(path, cv2.cvtColor(rgb_images, cv2.COLOR_RGB2BGR))  # OpenCV uses BGR


def save_depth_uint16(depth_maps: np.ndarray, output_dir: str, prefix: str = "depth"):
    """
    Saves Depth image.

    Args:
        depth_maps (np.ndarray): Depth Image
        output_dir (str): Directory to save the images.
        prefix (str): Optional prefix for filenames.
    """
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{prefix}.png")

    # Save as 16-bit PNG (supports uint16)
    cv2.imwrite(path, depth_maps.astype(np.uint16))
