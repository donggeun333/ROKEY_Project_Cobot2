from __future__ import annotations

from array import array as byte_array

import numpy as np


def bgr8_to_image_message(image, header=None):
    """Create a ROS bgr8 Image without relying on OpenCV type constants."""
    from sensor_msgs.msg import Image

    array = np.asarray(image)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError(
            f"bgr8 image must have shape (height, width, 3), got {array.shape}"
        )
    if array.dtype != np.uint8:
        raise ValueError(f"bgr8 image must use uint8, got {array.dtype}")

    array = np.ascontiguousarray(array)
    height, width, _ = array.shape
    message = Image()
    if header is not None:
        message.header = header
    message.height = height
    message.width = width
    message.encoding = "bgr8"
    message.is_bigendian = False
    message.step = width * 3
    # Assigning bytes makes generated ROS message setters validate every byte in
    # Python. An array('B') takes their optimized fast path instead.
    message.data = byte_array("B", array.tobytes())
    return message


def image_message_to_bgr8(message) -> np.ndarray:
    """Decode a ROS bgr8/rgb8 Image without importing cv2 or cv_bridge."""
    encoding = str(message.encoding).lower()
    if encoding not in {"bgr8", "rgb8"}:
        raise ValueError(
            f"dashboard only supports bgr8/rgb8 images, got {message.encoding!r}"
        )

    height = int(message.height)
    width = int(message.width)
    row_bytes = width * 3
    step = int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError(f"invalid image size: {width}x{height}")
    if step < row_bytes:
        raise ValueError(f"image step {step} is smaller than {row_bytes}")

    buffer = np.frombuffer(bytes(message.data), dtype=np.uint8)
    required = height * step
    if buffer.size < required:
        raise ValueError(
            f"image data has {buffer.size} bytes, expected at least {required}"
        )

    image = buffer[:required].reshape(height, step)
    image = image[:, :row_bytes].reshape(height, width, 3)
    if encoding == "rgb8":
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)
