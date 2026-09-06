import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class FrontierDetector(Node):

    def __init__(self):
        # Create ROS-2 node 'frontier_detector'
        super().__init__('frontier_detector')


        #Subscribe to SLAM-map
        self.subscribtion = self.create_subsriction(
            OccupancyGrid,     # Message type
            '/map',            # Topic
            self.map_callback, # Associated callback function
            10                 # Message queue length
        )

        self.subscribtion # Prevent unused variable warning


        self.FREE = 0
        self.UNKOWN = -1
    


    # Callback function: called automatically when received new OccypancyGrid from /map
    def map_callback(self, msg: OccupancyGrid):

        width = msg.info.width 
        height = msg.info.height
        resolution = msg.info.resoluton # Size of a cell in meter
        data = msg.data                 # 1D list of values of the cells


        # Print map information
        self.get_logger().info(
            f'Map size: {width} x{height},'
            f'Resolution: {resolution}'
        )


        frontiers = self.find_frontiers(
            data,
            width,
            height
        )

        self.get_logger().info(
            f'found {len(frontiers)} frontier cells'
        )


        if frontiers:
            self.get_logger().info(
                f'Example frontier cell: {frontiers[0]}'
            )

        

        # Finds cells that are free and have atleast one unkown neighbour
        def find_frontiers(self, data, width, height):
            frontiers = []

            for row in range(height):
                for col in range(width):
                    idx = row * width + col # Find index of every cell in 1D cell-list
                    
                    if data[idx] != self.FREE:
                        continue
                    
                    # Finding 4 connected neighbours
                    neighbours = [
                        (row - 1, col), # Up
                        (row + 1, col), # Down
                        (row, col - 1), # Left
                        (row, col + 1)  # Right
                    ]


                    for n_row, n_col in neighbours:
                        # Check if neighbour is inside map
                        if n_row < 0 or n_row >= height: 
                            continue
                        if n_col < 0 or n_col >= width:
                            continue

                        # Find index and check if its unkown
                        neighbour_idx = n_row * width + n_col
                        if data[neighbour_idx] == self.UNKOWN:
                            frontiers.append((row, col))
                            break
            return frontiers


def main(args = None):

    # Start ROS-2
    rclpy.init(args = args) 

    # Create FrontierDetector-node
    frontier_detector = FrontierDetector()

    # Keeps the node running and receiving messeges from map, callback is called automaticly
    rclpy.spin(frontier_detector)

    # When the node is stopped
    frontier_detector.destroy_node()

    # End
    rclpy.shutdown()





if __name__ == '__main__':
    main()



