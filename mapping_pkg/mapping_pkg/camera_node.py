"""Thin ROS 2 wrapper around aruco_core.

Jobs: subscribe, convert messages <-> NumPy with cv_bridge, call aruco_core,
publish the debug image, log. No detection maths belongs in this file.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from mapping_pkg import aruco_core


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # ---- Parameters -------------------------------------------------
        # Defaults match the Waffle Pi Gazebo model (turtlebot3_gazebo
        # models/turtlebot3_waffle_pi/model.sdf): sensor named "camera" with
        # libgazebo_ros_camera and no namespace -> /camera/image_raw and
        # /camera/camera_info, frame camera_rgb_optical_frame, 640x480.
        #
        # use_sim_time is NOT declared here: every rclpy node declares it
        # automatically. Set it from the launch file / --ros-args.
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        # If true, image_topic must point at a CompressedImage topic,
        # e.g. /camera/image_raw/compressed. The topic name is used verbatim.
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('debug_image_topic', '/aruco/debug_image')
        # 0.1 m verified from the world SDF: 0.1 x 0.1 box, texture is
        # black edge-to-edge (no white margin).
        self.declare_parameter('marker_size', aruco_core.DEFAULT_MARKER_SIZE_M)
        # DICT_6X6_1000 verified from the texture names 6x6_1000-{0,2,42}.png.
        self.declare_parameter('aruco_dictionary', aruco_core.DEFAULT_DICTIONARY)

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.dictionary_name = self.get_parameter('aruco_dictionary').value

        self.bridge = CvBridge()

        # Filled in by the first valid CameraInfo message.
        self.K = None
        self.dist = None

        # ---- QoS --------------------------------------------------------
        # qos_profile_sensor_data = BEST_EFFORT reliability, KEEP_LAST depth 5.
        #
        # A BEST_EFFORT subscriber is compatible with BOTH best-effort and
        # reliable publishers. A RELIABLE subscriber (the default when you
        # pass a plain integer depth) is NOT compatible with a best-effort
        # publisher, and ROS 2 gives you no error - you just receive nothing.
        # So best-effort is the "always connects" choice for sensor input, and
        # dropping an old frame is fine for us anyway.
        #
        # If no messages arrive, run:
        #     ros2 topic info -v /camera/image_raw
        # and compare the "QoS profile -> Reliability" of the Publisher(s)
        # against this node's Subscription entry. Also check the publisher
        # count is not 0 (wrong topic name) and the message type matches
        # (Image vs CompressedImage).
        #
        # Used for camera_info as well, for the same reason.
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)

        if self.use_compressed:
            self.create_subscription(
                CompressedImage, image_topic, self.image_callback, qos_profile_sensor_data)
        else:
            self.create_subscription(
                Image, image_topic, self.image_callback, qos_profile_sensor_data)

        # Publisher is RELIABLE (default), depth 1: reliable publishers match
        # both reliable and best-effort subscribers, so RViz connects whatever
        # its Image display's reliability setting is. Depth 1 because stale
        # debug frames are useless.
        self.debug_pub = self.create_publisher(Image, debug_image_topic, 1)

        self.get_logger().info(
            f'Listening: image={image_topic} (compressed={self.use_compressed}), '
            f'camera_info={camera_info_topic}; marker_size={self.marker_size} m, '
            f'dictionary={self.dictionary_name}; publishing {debug_image_topic}')

    def camera_info_callback(self, msg):
        if self.K is not None:
            return  # Intrinsics don't change; cache the first valid message only.

        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            # An uncalibrated / zeroed CameraInfo would make solvePnP garbage.
            self.get_logger().warn(
                'CameraInfo has non-positive focal length; ignoring it.',
                throttle_duration_sec=5.0)
            return

        self.K = K
        self.dist = np.array(msg.d, dtype=np.float64)  # may be empty; aruco_core handles that
        self.get_logger().info(
            f'Cached intrinsics from CameraInfo ({msg.width}x{msg.height}, '
            f'frame {msg.header.frame_id}): fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} '
            f'cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}, dist={list(self.dist)}')

    def image_callback(self, msg):
        if self.K is None:
            # Throttling uses wall-clock time, so this still rate-limits
            # correctly before /clock has started under use_sim_time.
            self.get_logger().warn(
                'No CameraInfo received yet; skipping image.', throttle_duration_sec=2.0)
            return

        # ---- message -> array
        try:
            if self.use_compressed:
                image_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # cv_bridge raises CvBridgeError or others on bad input
            self.get_logger().error(f'cv_bridge conversion failed: {exc}',
                                    throttle_duration_sec=2.0)
            return

        # ---- algorithm (all maths lives in aruco_core)
        detections = aruco_core.detect_markers(
            image_bgr, self.K, self.dist, self.marker_size, self.dictionary_name)
        annotated = aruco_core.draw_detections(
            image_bgr, detections, self.K, self.dist, self.marker_size)

        # ---- array -> message
        debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        # Reuse the source header: keeps the camera frame_id and, importantly
        # under use_sim_time, the original (simulated/bag) capture time rather
        # than "now". Later stages (tf2) will need that stamp to be right.
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

        # ---- logging
        # rclpy throttles per call site, so a throttled log inside a for-loop
        # would only ever show the first marker. Build one line per frame.
        # tvec is in the camera OPTICAL frame: z forward, x right, y down.
        if detections:
            parts = [
                f'id {d.marker_id}: {d.range_m:.2f} m '
                f'(optical x={d.tvec[0]:+.2f} y={d.tvec[1]:+.2f} z={d.tvec[2]:+.2f})'
                for d in detections
            ]
            self.get_logger().info('Detected ' + '; '.join(parts), throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
