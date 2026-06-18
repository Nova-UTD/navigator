import sys
import rclpy
from rclpy.node import Node
from frame_tf_serv.srv import FrameTF
from builtin_interfaces.msg import Time
import numpy as np

class FrameTFClient(Node):
    def __init__(self):
        super().__init__("FrameTFClient")
        self.client = self.create_client(FrameTF, 'frame_tf')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, trying again...')
        self.req = FrameTF.Request()
        
    def send_request(self, cam_to_world,cam_to_pixel,world_to_pixel,camera_name,stamp):
        self.req.cam_to_world = cam_to_world
        self.req.cam_to_pixel = cam_to_pixel
        self.req.world_to_pixel = world_to_pixel
        self.req.camera_name = camera_name
        self.stamp = stamp
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.response = self.future.result()
        self.deserialized = np.frombuffer(self.response.coords,dtype=float)
        return self.response.tf_success, self.deserialized
    

def main(args=None):
    rclpy.init(args=args)
    
    minimal_client = FrameTFClient()
    stamp = Time()
    example_time = rclpy.time.Time()
    example_time.seconds = 1778615106
    example_time.nanosec = 368472548
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4],example_time)
    minimal_client.get_logger().info(
        'Result of CAM_TO_WORLD: %d, CAM_TO_PIXEL: %d, WORLD_TO_PIXEL: %d, CAMERA: %s, Provided Time = %d seconds, %d nanoseconds' %
        (int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), sys.argv[4],example_time.seconds, example_time.nanosec))
    print(response)
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()