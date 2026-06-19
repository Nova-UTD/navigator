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
        
    def send_request(self, cam_to_world,cam_to_pixel,world_to_pixel,camera_name,stamp,x,y,z):
        """ packages and sends request to FrameTF service
        
        Args:
        cam_to_world (int either 0 or 1)
        cam_to_pixel (int either 0 or 1)
        world_to_pixel (int either 0 or 1)
        camera_name (string)
        stamp (Time)
        x (float32)
        y (float32)
        z (float32)
        world_to_pixel
        
        Returns:
            response.tf_success (bool)
            desserialized ()
        
        """
        self.req.cam_to_world = cam_to_world
        self.req.cam_to_pixel = cam_to_pixel
        self.req.world_to_pixel = world_to_pixel
        self.req.camera_name = camera_name
        self.stamp = stamp
        self.req.x = x
        self.req.y = y
        self.req.z = z
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.response = self.future.result()
        self.deserialized = np.frombuffer(self.response.coords,dtype=float)
        return self.response.tf_success, self.deserialized
    
    

def main(args=None):
    rclpy.init(args=args)
    
    minimal_client = FrameTFClient()
    stamp = Time()
    ## Example Time input (change it up as you see fit)
    example_time = rclpy.time.Time()
    example_time.seconds = 1781736843
    example_time.nanosec = 163943087
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4],example_time, int(sys.argv[5]),int(sys.argv[6]),int(sys.argv[7]))
    minimal_client.get_logger().info(
        'Result of CAM_TO_WORLD: %d, CAM_TO_PIXEL: %d, WORLD_TO_PIXEL: %d, CAMERA: %s, Provided Time = %d seconds, %d nanoseconds' %
        (int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4],example_time.seconds, example_time.nanosec))
    
    print(response)
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()