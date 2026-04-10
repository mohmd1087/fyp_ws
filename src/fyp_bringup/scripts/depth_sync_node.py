#!/usr/bin/env python3
"""
Depth sync node: republishes /camera/depth/camera_info with the same
timestamp as /camera/depth/image_raw so that depthimage_to_laserscan
can synchronize them properly.

The Orbbec Astra driver publishes depth image and camera_info with
different timestamps (~400ms apart), which causes depthimage_to_laserscan
to drop all messages and output NaN scans.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class DepthSyncNode(Node):
    def __init__(self):
        super().__init__('depth_sync_node')
        self._latest_info: CameraInfo | None = None

        # Subscribe to original topics
        self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self._info_cb,
            10,
        )
        self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self._image_cb,
            10,
        )

        # Republish camera_info with corrected timestamp
        self._info_pub = self.create_publisher(
            CameraInfo,
            '/camera/depth/camera_info_synced',
            10,
        )
        # Republish image (passthrough, unchanged)
        self._image_pub = self.create_publisher(
            Image,
            '/camera/depth/image_synced',
            10,
        )

        self.get_logger().info('Depth sync node started')

    def _info_cb(self, msg: CameraInfo):
        self._latest_info = msg

    def _image_cb(self, msg: Image):
        if self._latest_info is None:
            return

        # Use image's original timestamp for both — ensures exact match
        now = msg.header.stamp

        # Pass image through unchanged (already has correct stamp)
        synced_img = msg

        # Re-stamp camera_info with exact same stamp object as image
        synced_info = CameraInfo()
        synced_info.header.stamp.sec = now.sec
        synced_info.header.stamp.nanosec = now.nanosec
        synced_info.header.frame_id = self._latest_info.header.frame_id
        synced_info.height = self._latest_info.height
        synced_info.width = self._latest_info.width
        synced_info.distortion_model = self._latest_info.distortion_model
        synced_info.d = list(self._latest_info.d)
        synced_info.k = list(self._latest_info.k)
        synced_info.r = list(self._latest_info.r)
        synced_info.p = list(self._latest_info.p)
        synced_info.binning_x = self._latest_info.binning_x
        synced_info.binning_y = self._latest_info.binning_y
        synced_info.roi = self._latest_info.roi

        self._info_pub.publish(synced_info)
        self._image_pub.publish(synced_img)


def main():
    rclpy.init()
    node = DepthSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
