import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import pathlib
import numpy
import os
import cv2
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image
from IPython.display import display
import time

# Ros packages
from cv_bridge import CvBridge
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node

from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

while "models" in pathlib.Path.cwd().parts:
    os.chdir('..')
 
 
def load_model(model_name):
  base_url = 'http://download.tensorflow.org/models/object_detection/'
  model_file = model_name + '.tar.gz'
  model_dir = tf.keras.utils.get_file(
    fname=model_name, 
    origin=base_url + model_file,
    untar=True)
 
  model_dir = pathlib.Path(model_dir)/"saved_model"
  model = tf.saved_model.load(str(model_dir))
  return model
 
PATH_TO_LABELS = '/models/research/object_detection/data/mscoco_label_map.pbtxt'
category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
 
model_name = 'ssd_inception_v2_coco_2017_11_17'
detection_model = load_model(model_name)
 
def run_inference_for_single_image(model, image):
  image = np.asarray(image)
  # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
  input_tensor = tf.convert_to_tensor(image)
  # The model expects a batch of images, so add an axis with `tf.newaxis`.
  input_tensor = input_tensor[tf.newaxis,...]
 
  # Run inference
  model_fn = model.signatures['serving_default']
  output_dict = model_fn(input_tensor)
 
  # All outputs are batches tensors.
  # Convert to numpy arrays, and take index [0] to remove the batch dimension.
  # We're only interested in the first num_detections.
  num_detections = int(output_dict.pop('num_detections'))
  output_dict = {key:value[0, :num_detections].numpy() 
                 for key,value in output_dict.items()}
  output_dict['num_detections'] = num_detections
 
  # detection_classes should be ints.
  output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)
    
  # Handle models with masks:
  if 'detection_masks' in output_dict:
    # Reframe the the bbox mask to the image size.
    detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
              output_dict['detection_masks'], output_dict['detection_boxes'],
               image.shape[0], image.shape[1])      
    detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                       tf.uint8)
    output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()
     
  return output_dict

def show_inference(model, frame):
  #take the frame from webcam feed and convert that to array
  image_np = np.array(frame)
  # Actual detection.
     
  output_dict = run_inference_for_single_image(model, image_np)
  # Visualization of the results of a detection.
  vis_util.visualize_boxes_and_labels_on_image_array(
      image_np,
      output_dict['detection_boxes'],
      output_dict['detection_classes'],
      output_dict['detection_scores'],
      category_index,
      instance_masks=output_dict.get('detection_masks_reframed', None),
      use_normalized_coordinates=True,
      line_thickness=5)
 
  return(image_np)


# ~0.044 seconds
class objectDetection(Node):

    def __init__(self):
        super().__init__('Object_detection')

        camera_sub = self.create_subscription(
            Image, '/cameras/camera0', self.cameraCb, 1)

        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 10)

        self.result_pub = self.create_publisher(Image, "/cameras/objectDetection", 1)

        self.current_image: Image = None

        self.cv_bridge = CvBridge()


    def clockCb(self, msg: Clock):
        self.clock = msg.clock
        

    def cameraCb(self, image):
        self.current_image = image
        frame = self.cv_bridge.imgmsg_to_cv2(self.current_image)
        t1 = time.time()
        Imagenp = show_inference(detection_model, frame)
        Imagenp = self.cv_bridge.cv2_to_imgmsg(Imagenp)
        self.result_pub.publish(Imagenp)
        t2 = time.time()
        print(t2 - t1)
        

def main(args=None):
    rclpy.init(args=args)

    objectDetectionNode = objectDetection()
    rclpy.spin(objectDetectionNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()










