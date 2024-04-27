---
layout: default
title: Traffic Light Detector
nav_order: 1
parent: Perception
---

# Traffic Light Detector Node
{: .no_toc }

*Maintained by NOVA*

## Overview
The `TrafficLightDetectorNode` is responsible for detecting traffic lights in images captured by a camera and publishing the detections. It utilizes a pre-trained Faster R-CNN model for traffic light detection. Unfortunately, the model only works on vertical black traffic lights. This is due to lack of data on any other type of traffic light. I have attatched links to the kaggle resources that I have used to preproccess data, train the model, and store everything.

---

### Kaggle Resources:
- Model Training Kaggle Notebook- (https://www.kaggle.com/code/neilagrawal04/traffic-light-detection-pytorch-starter)
- Model Dataset - (https://kaggle.com/datasets/neilagrawal04/traffic-light-model/settings)
    - Model currently used 'fasterrcnn_resnet50_fpn.pth'
- Texas Traffic Light Dataset - (https://www.kaggle.com/datasets/neilagrawal04/texas-traffic-lights/data)
    - YOLO format

### In:
- **/cameras/camera0** [*Image*](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - This topic receives image messages from the camera. The node subscribes to this topic to receive input images for traffic light detection.

### Out:
- **/traffic_light/detections** [*TrafficLightDetection*](../messages.md#trafficlightdetection)
  - This message contains information about detected traffic lights, including their positions, sizes, labels, and confidence scores. The node publishes this message to inform other nodes about detected traffic lights. This node outputs 1 for green, 2 for yellow, and 3 for red.

---

### preprocess_image(self, image_data)
Preprocesses the input image data before feeding it to the model for inference. This involves converting the image to a PyTorch tensor and resizing it to the required dimensions.

### detect_traffic_lights(self, image_data)
Performs traffic light detection on the input image data using a pre-trained Faster R-CNN model. Postprocesses the model's predictions to extract bounding boxes, scores, and labels for detected traffic lights.

### postprocess_predictions(self, predictions)
Processes the model's predictions to extract bounding boxes, confidence scores, and labels for detected traffic lights.

### image_callback(self, msg)
Callback function triggered upon receiving an image message from the camera. Converts the image message to OpenCV format, runs traffic light detection, and publishes the detection results.
