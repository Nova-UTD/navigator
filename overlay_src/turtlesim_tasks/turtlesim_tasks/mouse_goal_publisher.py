import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import tkinter as tk

WORLD = 11.0
PIXELS = 550
SCALE = WORLD / PIXELS

class MouseGoalPublisher(Node):
    def __init__(self):
        super().__init__('mouse_goal_publisher')
        self.pub = self.create_publisher(Point, '/mouse_goal', 10)
        self.get_logger().info('Mouse window: click to set a target for turtle1')
        self.root = tk.Tk()
        self.root.title("Click-to-move (maps to turtlesim 11x11)")
        self.canvas = tk.Canvas(self.root, width=PIXELS, height=PIXELS, bg="white")
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_click)
        self.timer = self.create_timer(0.02, self._tk_poll)

    def _tk_poll(self):
        self.root.update_idletasks()
        self.root.update()

    def on_click(self, event):
        x = event.x * SCALE
        y = (PIXELS - event.y) * SCALE
        msg = Point(x=x, y=y, z=0.0)
        self.pub.publish(msg)
        self.get_logger().info(f"Clicked -> goal ({x:.2f}, {y:.2f})")
        r = 4
        self.canvas.create_oval(event.x-r, event.y-r, event.x+r, event.y+r)

def main():
    rclpy.init()
    node = MouseGoalPublisher()
    try:
        rclpy.spin(node)
    except tk.TclError:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
