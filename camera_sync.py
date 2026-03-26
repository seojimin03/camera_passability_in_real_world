from __future__ import annotations

from typing import Callable, Optional

import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage

import numpy as np

from .config import (
    SYNC_SLOP_SEC,
    TOPIC_COLOR_COMPRESSED,
    TOPIC_DEPTH_COMPRESSED,
    TOPIC_CAMERA_INFO,
)


class CameraSynchronizer:
    """
    Step 1: Color/Depth/CameraInfo를 ApproximateTimeSynchronizer로 동기화하고,
    CompressedImage를 OpenCV/Numpy로 디코딩한 뒤 파이프라인 콜백으로 전달합니다.
    """

    def __init__(
        self,
        callback: Callable[[np.ndarray, np.ndarray, CameraInfo], None],
        queue_size: int = 10,
        slop_sec: float = SYNC_SLOP_SEC,
        color_topic: str = TOPIC_COLOR_COMPRESSED,
        depth_topic: str = TOPIC_DEPTH_COMPRESSED,
        info_topic: str = TOPIC_CAMERA_INFO,
    ):
        self._callback = callback
        self._bridge = CvBridge()

        sub_color = message_filters.Subscriber(color_topic, CompressedImage)
        sub_depth = message_filters.Subscriber(depth_topic, CompressedImage)
        sub_info = message_filters.Subscriber(info_topic, CameraInfo)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_color, sub_depth, sub_info],
            queue_size=queue_size,
            slop=slop_sec,
        )
        self._sync.registerCallback(self._synced_cb)

    def _decode_images(
        self,
        color_msg: CompressedImage,
        depth_msg: CompressedImage,
    ) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        try:
            color_img = self._bridge.compressed_imgmsg_to_cv2(
                color_msg,
                desired_encoding="bgr8",
            )
        except Exception as e:
            rospy.logwarn(f"[camera_sync] color decode failed: {e}")
            return None, None

        try:
            depth_img = self._bridge.compressed_imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
        except Exception as e:
            rospy.logwarn(f"[camera_sync] depth decode failed: {e}")
            return None, None

        # depth는 보통 (H, W) 또는 (H, W, 1). (H, W)로 정규화
        if depth_img is not None and depth_img.ndim == 3 and depth_img.shape[-1] == 1:
            depth_img = depth_img[:, :, 0]

        return color_img, depth_img

    def _synced_cb(self, color_msg: CompressedImage, depth_msg: CompressedImage, info_msg: CameraInfo) -> None:
        color_img, depth_img = self._decode_images(color_msg, depth_msg)
        if color_img is None or depth_img is None:
            return

        self._callback(color_img, depth_img, info_msg)

