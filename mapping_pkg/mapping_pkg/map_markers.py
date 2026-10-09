"""
A node object that takes the detections from the camera node and publishes them as markers for RViz.

It should be run after the camera node, and it will subscribe to the camera node's output topic. 
It will then publish the markers to a topic that RViz can subscribe to.

It will update the markers in real time as the camera node detects new markers, and average frames over time to reduce noise in the marker positions.

This node should find the pose at time that the detection frame was taken, the gets the markers position relative to that. 
Then add them to get markers position on map

I've been going along with these
https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Adding-A-Frame-Py.html (i'm using PointStamp tho)
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.node import Time 

#i don't think we need these since we are getting the msg from camera_node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from mapping_pkg import aruco_core

#detection from camera node
from mapping_interfaces.msg import MarkerDetectionArray

#tf2 imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

MAX_RANGE = 2

class MapMarkersNode(Node):
    
    def __init__(self):
        super().__init__('map_markers_node')


        # ---- ROS Parameter ----------------------------------------------
        #needs SLAM running
        self.map_frame = self.declare_parameter('map_frame', 'map').value

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

        #Transform stuff, initialises buffer for transform storage and the listener fills it
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #subscriber for the camera node output, queue of 10 
        self.create_subscription(MarkerDetectionArray, '/aruco/detections', self.get_detected_markers, 10)
       


    #assigns map positions to aruco marker
    def get_detected_markers(self, msg):
        # maybe use a minimum distance or pixel area to filter out bad reads

        in_range = []
        from_frame = msg.header.frame_id
        to_frame = self.map_frame 
        frame_time = Time.from_msg(msg.header.stamp)

        if not msg.markers:
                    return 

        #Checks range of marker detection, bigger will have larger error
        for m in msg.markers:

            if m.range_m <= MAX_RANGE:
            
                in_range.append(m)

            else:
                return
            
        
        try: 
            # this gets the position of the camera when detection was made
            transform = self.tf_buffer.lookup_transform(
                        to_frame, 
                        from_frame, 
                        frame_time,
                        timeout=Duration(seconds=0.1))

        # just error message
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame} to {from_frame} : {ex}')
            return

        for mkr in msg.markers:

            cameraPoint = PointStamped()

            cameraPoint.header = msg.header
            cameraPoint.point = msg.position

            # camera to map transform
            mapPoint_stamped = do_transform_point(cameraPoint, transform)

            # x, y and z
            p = mapPoint_stamped

            self.update_markers(mkr.id, np.array([p.x, p.y, p.z]))
    
    
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

    def place_marker(self):

        out = MarkerDetectionArray()

        for marker_id, mk in self.markers.items():
            mkr = self.detected_markers()
            mkr.header.frame_id = self.map_frame
            mkr.id = marker_id
            mkr.type = self.detected_markers.SPHERE
            mkr.pose.position.x = self.detected_markers.p.x
            mkr.pose.position.y = self.detected_markers.p.z
            mkr.pose.position.z = 0
            mkr.scale.x = 0.1
            mkr.scale.y = 0.1
            mkr.scale.z = 0.1
            mkr.colour.r = 0
            mkr.colour.g = 1
            mkr.colour.b = 0
            mkr.colour.a = 1.0
            out.markers.append(mkr)

        self.rviz_pub.publish(out)

 
def main(args=None):
    rclpy.init(args=args)
    node = MapMarkersNode()
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
