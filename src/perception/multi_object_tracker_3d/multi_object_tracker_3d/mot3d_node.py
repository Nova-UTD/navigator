import rclpy
from rclpy.node import Node
from navigator_msgs.msg import Object3D, Object3DArray
from .tracker import Tracker3D
from easydict import EasyDict
from mmdet3d.structures import LiDARInstance3DBoxes
from geometry_msgs.msg import Point


CLASS2LABEL = {
    'Pedestrian': 0, 
    'Cyclist': 1, 
    'Car': 2        
}

LABEL2CLASS = {v: k for k, v in CLASS2LABEL.items()}

class MultiObjectTracker3DNode(Node):

    def __init__(self):
        super().__init__('multi_object_tracker_3d_node')
        
        self.declare_parameter("state_func_covariance", 100)
        self.declare_parameter("measure_func_covariance", 0.001)
        self.declare_parameter("prediction_score_decay", 0.03)
        self.declare_parameter("lidar_scanning_frequency", 10)
        self.declare_parameter("max_prediction_num", 12)
        self.declare_parameter("max_prediction_num_for_new_object", 2)
        self.declare_parameter("input_score_threshold", 0.5)
        self.declare_parameter("init_score_threshold", 0.5)
        self.declare_parameter("update_score_threshold", 0.0)
        self.declare_parameter("post_score_threshold", 0.0)
        self.declare_parameter("latency", 0)

        state_func_covariance = self.get_parameter("state_func_covariance").get_parameter_value().integer_value
        measure_func_covariance = self.get_parameter("measure_func_covariance").get_parameter_value().double_value
        prediction_score_decay = self.get_parameter("prediction_score_decay").get_parameter_value().double_value
        lidar_scanning_frequency = self.get_parameter("lidar_scanning_frequency").get_parameter_value().integer_value
        max_prediction_num = self.get_parameter("max_prediction_num").get_parameter_value().integer_value
        max_prediction_num_for_new_object = self.get_parameter("max_prediction_num_for_new_object").get_parameter_value().integer_value
        input_score_threshold = self.get_parameter("input_score_threshold").get_parameter_value().double_value
        init_score_threshold = self.get_parameter("init_score_threshold").get_parameter_value().double_value
        update_score_threshold = self.get_parameter("update_score_threshold").get_parameter_value().double_value
        post_score_threshold = self.get_parameter("post_score_threshold").get_parameter_value().double_value
        latency = self.get_parameter("latency").get_parameter_value().integer_value

        config = {
            # KF parameters
            'state_func_covariance': state_func_covariance,
            'measure_func_covariance': measure_func_covariance,
            'prediction_score_decay': prediction_score_decay,
            'LiDAR_scanning_frequency': lidar_scanning_frequency,

            # max prediction number of state function
            'max_prediction_num': max_prediction_num,
            'max_prediction_num_for_new_object': max_prediction_num_for_new_object,

            # detection score threshold
            'input_score': input_score_threshold,
            'init_score': init_score_threshold,
            'update_score': update_score_threshold,
            'post_score': post_score_threshold,

            # tracking latency (s)
            # -1: global tracking
            # 0.->500: online or near online tracking
            'latency': latency
        }

        # creat MOT for each class
        self.trackers = {}
        for label in LABEL2CLASS.keys():
            self.trackers[label] = Tracker3D(box_type="OpenPCDet", config=EasyDict(config))

        self.frame_stamp = 0
        self.label_seed = 0
        self.unique_ids = {}

        self.objects_subscription = self.create_subscription(
            msg_type = Object3DArray,
            topic = 'objdet3d_raw',
            callback = self.detection_callback, 
            qos_profile = 10
        )

        self.tracked_objects_publisher = self.create_publisher(
            msg_type = Object3DArray,
            topic = 'objdet3d_tracked',
            qos_profile = 10
        )

        

    def detection_callback(self, detection_msg: Object3DArray):
        # sort objects by class
        bounding_boxes = {}
        scores = {}
        for label in LABEL2CLASS.keys():
            bounding_boxes[label] = []
            scores[label] = []

        for object_instance in detection_msg.objects:
            bounding_boxes[object_instance.label].append(object_instance.bounding_box.coordinates)
            scores[object_instance.label].append(object_instance.confidence_score)

        # get tracked objects
        tracked_objects_array = Object3DArray()
        for label in LABEL2CLASS.keys():
            output_bounding_boxes, output_ids = self.trackers[label].tracking(
                bbs_3D = bounding_boxes[label], 
                scores = scores[label],
                timestamp = self.frame_stamp
            )
            output_corners = LiDARInstance3DBoxes(output_bounding_boxes).corners.cpu().numpy()

            for coordinates, corners, id in zip(output_bounding_boxes, output_corners, output_ids):
                tracked_object = Object3D()
                tracked_object.label = label
                tracked_object.id = self.get_unique_id(label, id)

                x, y, z, x_size, y_size, z_size, yaw = coordinates
                tracked_object.bounding_box.coordinates[0] = float(x)
                tracked_object.bounding_box.coordinates[1] = float(y)
                tracked_object.bounding_box.coordinates[2] = float(z)
                tracked_object.bounding_box.coordinates[3] = float(x_size)
                tracked_object.bounding_box.coordinates[4] = float(y_size)
                tracked_object.bounding_box.coordinates[5] = float(z_size)
                tracked_object.bounding_box.coordinates[6] = float(yaw)

                #corners[:, 2] += 1.5
                x0y0z0, x0y0z1, x0y1z1, x0y1z0, x1y0z0, x1y0z1, x1y1z1, x1y1z0 = corners
                tracked_object.bounding_box.corners[3] = Point(x=float(x0y0z0[0]), y=float(x0y0z0[1]), z=float(x0y0z0[2]))
                tracked_object.bounding_box.corners[2] = Point(x=float(x0y0z1[0]), y=float(x0y0z1[1]), z=float(x0y0z1[2]))
                tracked_object.bounding_box.corners[1] = Point(x=float(x0y1z1[0]), y=float(x0y1z1[1]), z=float(x0y1z1[2]))
                tracked_object.bounding_box.corners[0] = Point(x=float(x0y1z0[0]), y=float(x0y1z0[1]), z=float(x0y1z0[2]))
                tracked_object.bounding_box.corners[7] = Point(x=float(x1y0z0[0]), y=float(x1y0z0[1]), z=float(x1y0z0[2]))
                tracked_object.bounding_box.corners[6] = Point(x=float(x1y0z1[0]), y=float(x1y0z1[1]), z=float(x1y0z1[2]))
                tracked_object.bounding_box.corners[5] = Point(x=float(x1y1z1[0]), y=float(x1y1z1[1]), z=float(x1y1z1[2]))
                tracked_object.bounding_box.corners[4] = Point(x=float(x1y1z0[0]), y=float(x1y1z0[1]), z=float(x1y1z0[2]))

                tracked_objects_array.objects.append(tracked_object)

        tracked_objects_array.header.stamp = detection_msg.header.stamp
        tracked_objects_array.header.frame_id = detection_msg.header.frame_id
        self.tracked_objects_publisher.publish(tracked_objects_array)   
        self.frame_stamp += 1
    
    # remaps ids unique for each class to unique for all classes
    def get_unique_id(self, label, id):
        if (label, id) not in self.unique_ids:
            self.unique_ids[(label, id)] = self.label_seed
            self.label_seed += 1
        unique_id = self.unique_ids[(label, id)]
        return unique_id
    

def main(args=None):
    rclpy.init(args=args)

    mot3d_node = MultiObjectTracker3DNode()
    rclpy.spin(mot3d_node)

    mot3d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
