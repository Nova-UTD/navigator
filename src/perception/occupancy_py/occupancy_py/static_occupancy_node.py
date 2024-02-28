import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid
from navigator_msgs.msg import Masses

res = 1/3

class StaticOccupancyNodePy(Node):
    def __init__(self):
        super().__init__('static_occupancy_node_py')

        # Subscribe to and use CARLA's clock
        self.clock = Clock()  # Initialize with an empty Clock message
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_callback, 10)

        # Subscribe to the filtered lidar data
        self.pcd_sub = self.create_subscription(
            PointCloud2, '/lidar/filtered', self.point_cloud_callback, 10)

        # Publishers
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid, '/grid/occupancy/current', 10)
        self.masses_pub = self.create_publisher(
            Masses, '/grid/masses', 10)

    def clock_callback(self, msg):
        # Callback for clock subscriber
        self.clock = msg

    def point_cloud_callback(self, msg):
        # Callback for point cloud subscriber
        # Implement your logic for processing the point cloud data
        pass

    def point_cloud_cb(self, msg):
        # Converts the PCL ROS message using pcl_conversions.
        cloud = pcl.PointCloud()
        pcl.fromROSMsg(msg, cloud)

        # 1. Convert new measurement into a DST grid.
        self.create_occupancy_grid(cloud)

        # 2. Updates previous grid with updated grid values (important in cases of variable grid size)
        self.update_previous()

        # 3. Add decayed region (previous grid) to the updated grid
        self.mass_update()

        # 4. Publish static occupancy grid and mass grid
        self.publish_occupancy_grid()

        # 5. Clear current measured grid
        self.clear()

    def create_occupancy_grid(self, cloud):
        #Ray traces towards occupied spaces
        self.add_points_to_DST(cloud)
        #Ray traces rest of grid to fill with empty space
        self.add_free_spaces_to_DST()

    def add_points_to_DST(self, cloud):
        for point in cloud:
            x = point.x/res
            y = point.y/res
            z = point.z

            if (-1*z > .5):
                print("Point was above max height, skipping.")

            if x < (-1 * HALF_SIZE) or y < (-1 * HALF_SIZE) or x >= HALF_SIZE or y >= HALF_SIZE:
                # print("Point was outside grid boundaries, skipping.")
                continue

            if point.y > 0 and point.x < 0:
                angle = 180 - int(math.atan(abs(point.y) / abs(point.x)) * 180.0 / math.pi)
            elif point.y < 0 and point.x < 0:
                angle = 180 + int(math.atan(abs(point.y) / abs(point.x)) * 180.0 / math.pi)
            elif point.y < 0 and point.x > 0:
                angle = 360 - int(math.atan(abs(point.y) / abs(point.x)) * 180.0 / math.pi)
            else:
                angle = int(math.atan(abs(point.y) / abs(point.x)) * 180.0 / math.pi)

            angles[angle] = True

            slope = x/y

            if 0 < slope <= 1 and x > 0:
                self.ray_tracing_approximation_y_increment(x, y, 1, 1, False)
            elif slope > 1 and x > 0:
                self.ray_tracing_approximation_x_increment(x, y, 1, 1, False)
            elif -1 <= slope < 0 and x > 0:
                self.ray_tracing_approximation_y_increment(x, -y, 1, -1, False)
            elif slope < -1 and x > 0:
                self.ray_tracing_approximation_x_increment(x, -y, 1, -1, False)
            elif slope > 1 and x < 0:
                self.ray_tracing_approximation_x_increment(-x, -y, -1, -1, False)
            elif 0 < slope <= 1 and x < 0:
                self.ray_tracing_approximation_y_increment(-x, -y, -1, -1, False)
            elif -1 <= slope < 0 and x < 0:
                self.ray_tracing_approximation_y_increment(-x, y, -1, 1, False)
            elif slope < -1 and x < 0:
                self.ray_tracing_approximation_x_increment(-x, y, -1, 1, False)

    def add_free_spaces_to_the_DST(self):
        angles = [False] * 3600  # Initialize with False values for angles 0.0 to 359.9 degrees

        for i in range(3600):
            angle = i * 0.1

            if not angles[int(angle)]:
                x, y = 0, 0

                if 0.0 < angle <= 45.0:
                    x = 64
                    y = int(math.tan(math.radians(angle)) * x)
                elif 45.0 < angle < 90.0:
                    y = 64
                    x = int(y / math.tan(math.radians(angle)))
                elif 90.0 < angle <= 135.0:
                    y = 64
                    x = int(y / math.tan(math.radians(angle - 180.0)))
                elif 135.0 < angle < 180.0:
                    x = -64
                    y = int(math.tan(math.radians(angle - 180.0)) * x)
                elif 180.0 < angle <= 225.0:
                    x = -64
                    y = int(math.tan(math.radians(angle - 180.0)) * x)
                elif 225.0 < angle < 270.0:
                    y = -64
                    x = int(y / math.tan(math.radians(angle - 180.0)))
                elif 270.0 < angle <= 315.0:
                    y = -64
                    x = int(y / math.tan(math.radians(angle - 360.0)))
                elif 315.0 < angle < 360.0:
                    x = 64
                    y = int(math.tan(math.radians(angle - 360.0)) * x)
                elif angle == 0.0 or angle == 360.0:
                    self.ray_tracing_horizontal(64)
                    continue
                elif angle == 90.0:
                    self.ray_tracing_vertical(64)
                    continue
                elif angle == 180.0:
                    self.ray_tracing_horizontal_n(-64)
                    continue
                elif angle == 270.0:
                    self.ray_tracing_vertical_n(-64)
                    continue

                if -64 <= x <= 64 and -64 <= y <= 64:
                    slope = y / x
                    if 0 < slope <= 1 and x > 0:
                        self.ray_tracing_approximation_y_increment(x, y, 1, 1, True)
                    elif slope > 1 and x > 0:
                        self.ray_tracing_approximation_x_increment(x, y, 1, 1, True)
                    elif -1 <= slope < 0 and x > 0:
                        self.ray_tracing_approximation_y_increment(x, -y, 1, -1, True)
                    elif slope < -1 and x > 0:
                        self.ray_tracing_approximation_x_increment(x, -y, 1, -1, True)
                    elif slope > 1 and x < 0:
                        self.ray_tracing_approximation_x_increment(-x, -y, -1, -1, True)
                    elif 0 < slope <= 1 and x < 0:
                        self.ray_tracing_approximation_y_increment(-x, -y, -1, -1, True)
                    elif -1 <= slope < 0 and x < 0:
                        self.ray_tracing_approximation_y_increment(-x, y, -1, 1, True)
                    elif slope < -1 and x < 0:
                        self.ray_tracing_approximation_x_increment(-x, y, -1, 1, True)

                angles[int(angle)] = False

    def add_ego_mask(self):
        # Vehicle shape.
        for i in range(60, 68):
            for j in range(62, 67):
                measured_occ[i][j] = 1.0
                measured_free[i][j] = 0.0

    def publish_occupancy_grid(self):
        # Occupancy Grid
        msg = OccupancyGrid()

        msg.header.stamp = self.clock.clock
        msg.header.frame_id = "base_link"  # TODO: Make sure the frame is the correct one.
        msg.info.resolution = res
        msg.info.width = GRID_SIZE
        msg.info.height = GRID_SIZE
        msg.info.origin.position.z = 0.2
        msg.info.origin.position.x = -64.0 * (1. / 3.)
        msg.info.origin.position.y = -64.0 * (1. / 3.)

        # Masses
        masses_msg = Masses()
        masses_msg.occ.clear()
        masses_msg.free.clear()
        masses_msg.width = GRID_SIZE
        masses_msg.height = GRID_SIZE

        probabilities = self.get_grid_cell_probabilities()

        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                msg.data.append(100 * probabilities[j][i])
                masses_msg.occ.append(updated_occ[i][j])
                masses_msg.free.append(updated_free[i][j])

        occupancy_grid_pub.publish(msg)
        masses_pub.publish(masses_msg)


    def update_previous(self):
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                previous_free[i][j] = updated_free[i][j]
                previous_occ[i][j] = updated_occ[i][j]

    def mass_update(self):
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                updated_occP[i][j] = min(decay_factor * previous_occ[i][j], 1.0 - previous_free[i][j])
                updated_freeP[i][j] = min(decay_factor * previous_free[i][j], 1.0 - previous_occ[i][j])

        # Combine measurement and prediction to form posterior occupied and free masses.
        self.update_of()


    def update_of(self):
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                # probability of cell being unknown
                unknown_pred = 1.0 - updated_freeP[i][j] - updated_occP[i][j]

                # probability of measured cell being unknown
                measured_cell_unknown = 1.0 - measured_free[i][j] - measured_occ[i][j]

                # normalizing factor, ensures probabilities for occupancy/free add up to 1
                k_value = updated_freeP[i][j] * measured_occ[i][j] + updated_occP[i][j] * measured_free[i][j]

                updated_occ[i][j] = (updated_occP[i][j] * measured_cell_unknown + unknown_pred * measured_occ[i][j] + updated_occP[i][j] * measured_occ[i][j]) / (1.0 - k_value)
                updated_free[i][j] = (updated_freeP[i][j] * measured_cell_unknown + unknown_pred * measured_free[i][j] + updated_freeP[i][j] * measured_free[i][j]) / (1.0 - k_value)

    def get_grid_cell_probabilities(self):
        cell_probabilities = []
        for i in range(GRID_SIZE):
            row = []
            for j in range(GRID_SIZE):
                probability = 0.5 * updated_occ[i][j] + 0.5 * (1.0 - updated_free[i][j])
                row.append(probability)
            cell_probabilities.append(row)

        return cell_probabilities


    def find_nearest(self, num, value, min_val, max_val, res):
        index = int(num * (value - min_val + res / 2.0) / (max_val - min_val + res))
        return index

    def ray_tracing_approximation_y_increment(self, x2, y2, flip_x, flip_y, inclusive):
        x1, y1 = 0, 0
        slope = 2 * (y2 - y1)
        slope_error = slope - (x2 - x1)
        
        for x in range(x1, x2):
            if measured_occ[flip_x * x + 64][flip_y * y + 64] == meas_mass:
                break
            measured_free[flip_x * x + 64][flip_y * y + 64] = meas_mass
            slope_error += slope
            if slope_error >= 0:
                y += 1
                slope_error -= 2 * (x2 - x1)

        if not inclusive:
            x_coordinate = flip_x * x2 + 64
            y_coordinate = flip_y * y2 + 64
            measured_occ[x_coordinate][y_coordinate] = meas_mass
            measured_free[x_coordinate][y_coordinate] = 0.0


    def ray_tracing_approximation_x_increment(self, x2, y2, flip_x, flip_y, inclusive):
        x1, y1 = 0, 0
        slope = 2 * (x2 - x1)
        slope_error = slope - (y2 - y1)

        for x in range(x1, x2):
            if measured_occ[flip_x * x + 64][flip_y * y + 64] == meas_mass:
                break
            measured_free[flip_x * x + 64][flip_y * y + 64] = meas_mass
            slope_error += slope
            if slope_error >= 0:
                x += 1
                slope_error -= 2 * (y2 - y1)

        if not inclusive:
            x_coordinate = flip_x * x2 + 64
            y_coordinate = flip_y * y2 + 64
            measured_occ[x_coordinate][y_coordinate] = meas_mass
            measured_free[x_coordinate][y_coordinate] = 0.0


    def ray_tracing_vertical(self, x2):
        x1, y1 = 0, 0
        x2 -= 1

        for x in range(x1, x2 + 1):
            if measured_occ[64][x + 64] == meas_mass:
                print("BROKE! VERTICAL + \n\n")
                break
            measured_free[x + 64][64] = meas_mass

        measured_free[x2 + 64][64] = 0.0


    def ray_tracing_vertical_n(self, x1):
        x2, y2 = 0, 0
        x1 += 1

        for x in range(x1, x2 + 1):
            if measured_occ[64][x + 64] == meas_mass:
                print("BROKE! VERTICAL - \n\n")
                break
            measured_free[x + 64][64] = meas_mass

        measured_free[x2 + 64][64] = 0.0


    def ray_tracing_horizontal(self, y2):
        x1, y1 = 0, 0
        y2 -= 1

        for y in range(y1, y2 + 1):
            if measured_occ[64][y + 64] == meas_mass:
                print("BROKE! HORIZONTAL + \n\n")
                break
            measured_free[64][y + 64] = meas_mass


    def ray_tracing_horizontal_n(self, y1):
        x1, y2 = 0, 0
        y1 += 1

        for y in range(y1, y2 + 1):
            if measured_occ[64][y + 64] == meas_mass:
                print("BROKE! HORIZONTAL - \n\n")
                break
            measured_free[64][y + 64] = meas_mass

        measured_free[64][y2 + 64] = 0.0


    def clear(self):
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                measured_occ[i][j] = 0.0
                measured_free[i][j] = 0.0

def main():
    rclpy.init()

    node = StaticOccupancyNodePy()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
