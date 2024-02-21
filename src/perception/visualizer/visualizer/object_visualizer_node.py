import rclpy
from rclpy.node import Node
from navigator_msgs.msg import Object3D, Object3DArray
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration

# label to color mappings, RGB
LABEL_TO_COLOR = {
    0: [1.0, 0.0, 0.0],     # Pedestrian
    1: [0.0, 1.0, 0.0],     # Cyclist
    2: [0.0, 0.0, 1.0]      # Car        
}

class ObjectVisualizerNode(Node):

    def __init__(self):
        super().__init__('object_visualizer_node')

        self.subscription = self.create_subscription(
            msg_type = Object3DArray,
            topic = 'objdet3d_tracked',
            callback = self.visualize_objects,
            qos_profile = 1
        )

        self.visualization_publisher = self.create_publisher(MarkerArray, 'objdet3d_viz', 10)


    def visualize_objects(self, msg: Object3DArray):
        #self.get_logger().info(f"{msg.header}")
        
        marker_array = MarkerArray()
        for object in msg.objects:
            
            box = Marker()
            box.header.frame_id = msg.header.frame_id
            box.header.stamp = self.get_clock().now().to_msg()
            box.id = object.id
            box.type = 5
            box.color.r, box.color.g, box.color.b = LABEL_TO_COLOR[object.label]
            box.color.a = 1.0
            box.scale.x = 0.10
            box.lifetime = Duration(seconds=0.2).to_msg()    # should be removed when object.id exists
            box.ns = "object_bounding_box"

            for i in range(4):
                # this should do 0-1, 1-2, 2-3, 3-4
                src = object.bounding_box.corners[i]
                dst = object.bounding_box.corners[(i+1) % 4]
                box.points.append(src)
                box.points.append(dst)

                # this should do 4-5, 5-6, 6-7, 7-4
                src = object.bounding_box.corners[i+4]
                dst = object.bounding_box.corners[((i+1) % 4) + 4]
                box.points.append(src)
                box.points.append(dst)

                # this should do 0-4, 1-5, 2-6, 3-7
                src = object.bounding_box.corners[i]
                dst = object.bounding_box.corners[i+4]
                box.points.append(src)
                box.points.append(dst)

            marker_array.markers.append(box)

            tag = Marker()
            tag.header.frame_id = msg.header.frame_id
            tag.header.stamp = self.get_clock().now().to_msg()
            tag.id = object.id
            tag.type = 9
            tag.color.r, tag.color.g, tag.color.b = (1.0, 1.0, 1.0)
            tag.color.a = 1.0
            tag.scale.z = 0.5
            tag.lifetime = Duration(seconds=0.2).to_msg()   
            tag.ns = "object_tag"
            tag.pose.position.x = object.bounding_box.coordinates[0]
            tag.pose.position.y = object.bounding_box.coordinates[1]
            tag.pose.position.z = object.bounding_box.coordinates[2] + 1.5 * object.bounding_box.coordinates[5]
            tag.text = f'id:{object.id}'

            marker_array.markers.append(tag)

        self.visualization_publisher.publish(marker_array)
        


def main(args=None):
    rclpy.init(args=args)


    object_visualizer_node = ObjectVisualizerNode()

    rclpy.spin(object_visualizer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_visualizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
