import readchar
import rclpy

def keyboard_listener_thread(node):
    """
    Listens for the 's' key to be pressed in the terminal.
    """
    print("Listener active. Press 's' to save and exit.")
    try:
      while rclpy.ok():
          key = readchar.readkey()
          if key == 's':
              print("'s' key pressed!")
              node.trigger_shutdown_and_slam()
              break
    
    except KeyboardInterrupt:
        print("keyboardInterrupt received, quitting program. If node doesn't shutdown, try ctrl+C again.")