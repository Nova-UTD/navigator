---
layout: default
title: Pedestrian Skeleton Detector
nav_order: 7
parent: Perception
---

# Pedestrian Skeleton Detector

{: .no_toc }

_Maintained by NOVA_

## Overview

A ROS Node that uses YOLOv8 Pose Detection to detect pedestrian skeletons in real-time and publish the annotated results.

---

### In:

- **/cameras/camera0** [_Image_](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)
  - Receives an image from the camera (topic: `cameras/camera0`).

### Out:

- **/processed_image** [_Image_](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)

  - Publishes the processed image with pose annotations (topic: `processed_image`).

- **/detection_status** [_String_](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html)
  - Publishes the detection status, including confidence in detecting pedestrians.

---

## Function Descriptions

### image_callback(self, msg: Image)

Processes an incoming camera image, applies YOLOv8 Pose Detection, and republishes the annotated image.

#### Steps:

1. Converts the incoming ROS Image message into an OpenCV image using `CvBridge`.
2. Runs YOLOv8 Pose Detection on the image.
3. Extracts the detected pedestrian(s) and checks if confidence is above the threshold.
4. If a pedestrian is detected, it publishes a message to the `/detection_status` topic.
5. Annotates the image with pose keypoints.
6. Converts the annotated image back into a ROS Image message.
7. Publishes the annotated image to the `/processed_image` topic.
8. Optionally, displays the processed image in a window for debugging.

#### Error Handling:

- If an unsupported image encoding is detected, logs an error message.
- If an exception occurs during processing, logs the error details.

---

## Dependencies

- `rclpy`: ROS 2 Python API
- `sensor_msgs.msg.Image`: ROS Image message type
- `std_msgs.msg.String`: ROS String message type for detection status
- `cv_bridge`: Converts between ROS Image messages and OpenCV images
- `cv2`: OpenCV for image processing
- `ultralytics.YOLO`: YOLOv8 model for pose detection
- `numpy`: Array operations

---

## Usage

Launch the node and subscribe to `/processed_image` to receive annotated images and `/detection_status` for detection updates:

```bash
ros2 run <package_name> <node_executable>
```
