from __future__ import annotations

from typing import List, Tuple

import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import PointCloud2, PointField

from .config import BASE_LINK_FRAME, TOPIC_DYNAMIC_OBSTACLES, TOPIC_PASSABLE
from .tf_transformer import RobotPoint


class ObstaclePublisher:
    """
    Step 5: dynamic 장애물 PointCloud2 + passable Bool 동시 발행
    """

    def __init__(self, queue_size: int = 10):
        self._pub_obstacles = rospy.Publisher(
            TOPIC_DYNAMIC_OBSTACLES,
            PointCloud2,
            queue_size=queue_size,
        )
        self._pub_passable = rospy.Publisher(
            TOPIC_PASSABLE,
            Bool,
            queue_size=queue_size,
        )

        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    def publish(
        self,
        header,
        robot_points: List[RobotPoint],
        wall_pts: List[Tuple[float, float, float]],
        passable: bool,
    ) -> None:
        # dynamic 객체 점 + 가상 벽 점 합산
        combined_pts: List[Tuple[float, float, float]] = [
            (p.x, p.y, 0.0) for p in robot_points
        ] + wall_pts

        out_header = Header()
        out_header.stamp = header.stamp
        out_header.frame_id = BASE_LINK_FRAME

        cloud_msg = pc2.create_cloud(out_header, self._fields, combined_pts)
        self._pub_obstacles.publish(cloud_msg)

        passable_msg = Bool()
        passable_msg.data = bool(passable)
        self._pub_passable.publish(passable_msg)

        rospy.loginfo_throttle(
            1.0,
            f"[obstacle_publisher] objects={len(robot_points)} "
            f"wall_pts={len(wall_pts)} passable={passable}",
        )

