import sys
import rclpy
from rclpy.node import Node
from frame_tf_serv.srv import FrameTF
from builtin_interfaces.msg import Time
import numpy as np

class FrameTFClient(Node):
    def __init__(self):
        super().__init__("FrameTFClient")
        
        # initialize client
        self.client = self.create_client(FrameTF, 'frame_tf')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, trying again...')
        self.req = FrameTF.Request()
        
    def send_request(
        self,
        cam_to_world,
        cam_to_pixel,
        world_to_pixel,
        pixel_to_world,
        camera_name,
        stamp,
        x,
        y,
        z,
    ):
        """ packages and sends request to FrameTF service
        
        Args:
        cam_to_world (int either 0 or 1)
        cam_to_pixel (int either 0 or 1)
        world_to_pixel (int either 0 or 1)
        pixel_to_world (int either 0 or 1)
        camera_name (string)
        stamp (Time)
        x (float)  — world X, or pixel u for pixel_to_world
        y (float)  — world Y, or pixel v for pixel_to_world
        z (float)  — world Z (unused for pixel_to_world)
        
        Returns:
            response.tf_success (bool)
            deserialized (np.ndarray)
        
        """
        self.req.cam_to_world = cam_to_world
        self.req.cam_to_pixel = cam_to_pixel
        self.req.world_to_pixel = world_to_pixel
        self.req.pixel_to_world = pixel_to_world
        self.req.camera_name = camera_name
        self.req.stamp = stamp
        self.req.x = float(x)
        self.req.y = float(y)
        self.req.z = float(z)
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.response = self.future.result()
        if self.response.coords:
            self.deserialized = np.frombuffer(self.response.coords, dtype=np.float64)
        else:
            self.deserialized = np.array([])
        return self.response.tf_success, self.deserialized
    
    

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 9:
        print(
            "Usage: ros2 run frame_tf_client client "
            "<cam_to_world> <cam_to_pixel> <world_to_pixel> <pixel_to_world> "
            "<camera_name> <x> <y> <z>"
        )
        print(
            "pixel_to_world example: "
            "0 0 0 1 rgb_center 512.0 256.0 0.0"
        )
        rclpy.shutdown()
        return

    minimal_client = FrameTFClient()
    stamp = Time()  # 0/0 → service falls back to cloud stamp
    response = minimal_client.send_request(
        int(sys.argv[1]),
        int(sys.argv[2]),
        int(sys.argv[3]),
        int(sys.argv[4]),
        sys.argv[5],
        stamp,
        float(sys.argv[6]),
        float(sys.argv[7]),
        float(sys.argv[8]),
    )
    minimal_client.get_logger().info(
        'Result of CAM_TO_WORLD: %d, CAM_TO_PIXEL: %d, WORLD_TO_PIXEL: %d, '
        'PIXEL_TO_WORLD: %d, CAMERA: %s, x=%.2f y=%.2f z=%.2f'
        % (
            int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]),
            sys.argv[5], float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]),
        )
    )
    
    print(response)
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
