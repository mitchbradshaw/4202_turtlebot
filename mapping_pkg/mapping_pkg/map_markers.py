"""
A node object that takes the detections from the camera node and publishes them as markers for RViz.

It should be run after the camera node, and it will subscribe to the camera node's output topic. 
It will then publish the markers to a topic that RViz can subscribe to.

It will update the markers in real time as the camera node detects new markers, and average frames over time to reduce noise in the marker positions.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from mapping_pkg import aruco_core

class MapMarkersNode(Node):
    
    def __init__(self):
        super().__init__('map_markers_node')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('marker_size', aruco_core.DEFAULT_MARKER_SIZE_M)
        self.declare_parameter('aruco_dictionary', aruco_core.DEFAULT_DICTIONARY)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('debug_image_topic', '/aruco/debug_image')

        self.marker_size = float(self.get_parameter('marker_size').value)
        self.dictionary_name = self.get_parameter('aruco_dictionary').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize marker storage
        self.detected_markers = {}
      
    
    def update_markers(self, marker_id, position):
        """
        Update the detected markers with new positions.
        If the marker is already detected, average the new position with the old one.
        """
        if marker_id in self.detected_markers:
            old_position = self.detected_markers[marker_id]
            new_position = (old_position + position) / 2
            self.detected_markers[marker_id] = new_position
        else:
            self.detected_markers[marker_id] = position


    def get_detected_markers(self):
        """
        Return the current detected markers.
        """
        return self.detected_markers