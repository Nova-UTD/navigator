import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')
        self.bridge = CvBridge()

        #subscribe to segmentation and depth masks
        self.create_subscription(Image, '/segmentation_mask', self.process_segmentation, 10)
        self.create_subscription(Image, '/processed_depth', self.process_depth, 10)

        #publisher for occupancy grid
        self.publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        self.segmentation_mask = None
        self.depth_image = None

        print("Occupancy Grid Node Started!")
    
    def process_segmentation(self, msg):
        #receive segmentation mask
        self.segmentation_mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
        print(f"Segmentation Mask Received: Shape={self.segmentation_mask.shape}, Min={np.min(self.segmentation_mask)}, Max={np.max(self.segmentation_mask)}")
        self.generate_occupancy_grid()

    def process_depth(self, msg):
        #receive depth map
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        print(f"Depth Image Received: Shape={self.depth_image.shape}, Min={np.min(self.depth_image)}, Max={np.max(self.depth_image)}")
        self.generate_occupancy_grid()

    def generate_occupancy_grid(self):
        if self.segmentation_mask is None or self.depth_image is None:
            print("Segmentation or Depth Image Not Available Yet!")
            return

        #create empty grid of -1s
        grid_size = self.segmentation_mask.shape  # Match segmentation mask size
        resolution = 0.2
        occupancy_grid = np.ones(grid_size, dtype=np.int8) * -1  # Initialize to unknown (-1)

        print(f"Generating Occupancy Grid: Size={grid_size}")

        found_drivable = False  # Track if any drivable surface was found

        #loop through each grid cell
        for y in range(grid_size[0]):
            for x in range(grid_size[1]):
                seg_value = self.segmentation_mask[y, x]
                depth_value = self.depth_image[y, x]
                if seg_value == 255:  # if segmented value is drivable (white)
                    found_drivable = True  #at least one drivable point exists

                    if depth_value < 10.0:  #free space
                        occupancy_grid[y, x] = 0
                    elif depth_value >= 3.0:  #occupied space
                        occupancy_grid[y, x] = 100

        # Debugging: Check if we found any drivable surface
        if not found_drivable:
            print("WARNING: No drivable pixels found in segmentation mask!")

        print(f"Occupancy Grid Values: Min={np.min(occupancy_grid)}, Max={np.max(occupancy_grid)}")

        # Create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width = grid_size[1]
        msg.info.height = grid_size[0]
        msg.data = occupancy_grid.flatten().tolist()

        self.publisher.publish(msg)
        print("Published Occupancy Grid.")



def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()