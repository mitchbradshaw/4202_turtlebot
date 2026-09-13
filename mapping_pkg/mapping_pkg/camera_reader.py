from nav2_msgs import msg
import rclpy
from rclpy.node import Node

from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, CompressedImage

class CameraReader(Node):
    def __init__(self):
        super().__init__('camera_reader') # this defines the node name
        # Create a subscriber to the camera_info topic
        # The constructor inputs are (message type, topic name, associated
        # callback function, message queue length),→
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10)
        self.subscription # prevent unused variable warning
      
        # Create a publisher for the image topic
        # The constructor inputs are (message type, topic name, message queue
        #clength),→
        self.publisher_ = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            10)

    def camera_info_callback(self, msg:CameraInfo):
        # Log the camera info message to the console
        self.get_logger().info('Received camera info: %s' % msg)



def main(args=None):
    rclpy.init(args=args)

    # Create a new WaypointCycler node
    cam_reader = CameraReader()
    # Execute the node
    rclpy.spin(cam_reader)


if __name__ == '__main__':
    main()