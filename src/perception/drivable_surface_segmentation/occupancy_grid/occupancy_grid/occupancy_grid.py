import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
from skimage.draw import line
import yaml

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')
        self.bridge = CvBridge()

        # Set up the global config file
        self.declare_parameter('global_config', 'temp_value')
        self.file_path = self.get_parameter('global_config').value

        #subscribe to segmentation and depth masks
        self.create_subscription(Image, '/segmentation_mask', self.process_segmentation, 10)
        self.create_subscription(Image, '/processed_depth', self.process_depth, 10)

        #publisher for occupancy grid
        self.publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        self.segmentation_mask = None
        self.depth_image = None

        print("Occupancy Grid Node Started!")

    def process_segmentation(self, msg):
        self.segmentation_mask = self.bridge.imgmsg_to_cv2(msg, "mono8")
        print(f"Segmentation Mask Received: Shape={self.segmentation_mask.shape}, Min={np.min(self.segmentation_mask)}, Max={np.max(self.segmentation_mask)}")
        self.generate_occupancy_grid()

    def process_depth(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        print(f"Depth Image Received: Shape={self.depth_image.shape}, Min={np.min(self.depth_image)}, Max={np.max(self.depth_image)}")
        self.generate_occupancy_grid()

    def raycast_fill(self, grid, origin, free_cells):
        for fx, fy in free_cells:
            rr, cc = line(origin[1], origin[0], fy, fx)
            for y, x in zip(rr, cc):
                if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
                    grid[y, x] = 0  # mark as free
        return grid

    def generate_occupancy_grid(self):
        if self.segmentation_mask is None or self.depth_image is None:
            print("Segmentation or Depth Image Not Available Yet!")
            return

        # Open the config file
        try:
            with open(self.file_path, 'r') as file:
                data = yaml.safe_load(file)
        except FileNotFoundError:
            print("Error: config.yaml not found.")
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file: {e}")

        grid_size = self.segmentation_mask.shape
        resolution = data['occupancy_grids']['resolution']
        occupancy_grid = np.ones(grid_size, dtype=np.int8) * -1

        origin = (grid_size[1] // 2, 0)
        free_cells = []
        occupied_cells = []

        for y in range(grid_size[0]):
            for x in range(grid_size[1]):
                seg_value = self.segmentation_mask[y, x]
                depth_value = self.depth_image[y, x]

                if depth_value <= 0:
                    continue

                zCoord = depth_value
                xCoord = (x - 512) * zCoord / 470

                u = int((xCoord + (grid_size[1] * resolution / 2)) / resolution)
                v = int(zCoord / resolution)

                if 0 <= u < grid_size[1] and 0 <= v < grid_size[0]:
                    if seg_value > 0:
                        free_cells.append((u, v))
                    else:
                        occupied_cells.append((u, v))

        occupancy_grid = self.raycast_fill(occupancy_grid, origin, free_cells)

        for u, v in occupied_cells:
            if 0 <= u < grid_size[1] and 0 <= v < grid_size[0]:
                occupancy_grid[v, u] = 1

        # Resize occupancy grid to match size specified in config file
        prepend_data = [-1] * int(data['occupancy_grids']['width'] * data['occupancy_grids']['vehicle_longitudinal_location'])
        new_grid = np.insert(occupancy_grid, 0, prepend_data)

        if new_grid.shape[0] != data['occupancy_grids']['length']:
            diff = (data['occupancy_grids']['length'] - new_grid.shape[0]) / data['occupancy_grids']['resolution']
            diff = int(diff)  # Convert to integer
            
            if diff < 0:
                new_grid = new_grid[:int(diff * new_grid.shape[1])]
            elif diff > 0:
                new_grid.extend([-1] * int(diff * new_grid.shape[1]))
            
            grid_size[0] = int(data['occupancy_grids']['length'])

        if new_grid.shape[1] != data['occupancy_grids']['width']:
            diff = (data['occupancy_grids']['width'] - new_grid.shape[1]) / data['occupancy_grids']['resolution']     # Num cells to add or remove from each row
            diff = int(diff)  # Convert to integer

            if diff < 0:
                new_new_grid = [-1] * int(data['occupancy_grids']['width'] * new_grid.shape[0])
                offset = int(diff / 2 * -1)
                
                for i in range(int(new_grid.shape[0])):
                    start = int(i * data['occupancy_grids']['width'] + offset)
                    end = int(((i + 1) * data['occupancy_grids']['width']) - 1 - offset)

                    new_new_grid[int(i * data['occupancy_grids']['width']):int((i + 1) * data['occupancy_grids']['width'] - 1)] = new_grid[start:end]

                new_grid = new_new_grid
            elif diff > 0:
                new_new_grid = [-1] * int(data['occupancy_grids']['width'] * new_grid.shape[0])
                offset = int(diff / 2)
                for i in range(int(new_grid.shape[0])):
                    start = int(i * data['occupancy_grids']['width'] + offset)
                    end = int(((i + 1) * data['occupancy_grids']['width']) - 1 - offset)
                    new_new_grid[start:end] = new_grid[int(i * new_grid.shape[1]):int((i + 1) * new_grid.shape[1])]
                new_grid = new_new_grid

            grid_size[1] = int(data['occupancy_grids']['width'])


        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width = grid_size[1]
        msg.info.height = grid_size[0]
        msg.data = new_grid.flatten().tolist()
        msg.info.origin.position.x = -1 * data['occupancy_grids']['vehicle_latitudinal_location']
        msg.info.origin.position.y = -1 * data['occupancy_grids']['vehicle_longitudinal_location']

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