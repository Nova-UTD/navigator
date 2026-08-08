import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class Spawner(Node):
    def __init__(self):
        super().__init__('spawn_second')
        self.cli = self.create_client(Spawn, '/spawn')
        self.timer = self.create_timer(0.5, self.try_spawn)

    def try_spawn(self):
        if not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Waiting for /spawn...')
            return
        req = Spawn.Request()
        req.x = 8.0; req.y = 8.0; req.theta = 0.0; req.name = 'turtle2'
        fut = self.cli.call_async(req)
        fut.add_done_callback(self._done)
        self.timer.cancel()

    def _done(self, fut):
        try:
            res = fut.result()
            self.get_logger().info(f"Spawned: {res.name}")
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")

def main():
    rclpy.init()
    node = Spawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
