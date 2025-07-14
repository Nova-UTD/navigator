The code is provided. **There are a few problems**:

1. Substantial code is run in the callback functions. Instead, the data should only be stored during the callback, and processing should only be done on a timer.

2. The code is hardcoded to work on the rgb_center camera on CARLA. In order to run it on rosbags, the transform for the camera is needed.

3. Currently, my code subscribes to an Image topic (“semantics/semantic0”) for segmentation data, but I believe Sai has created an OccupancyGrid topic.

4. I created a couple of parameters with default values so the code could be used on other topics depending on the launch, but the topics are currently still hard-coded.

5. I have a test visualization in cv2 currently. This should of course be removed or moved to rviz

**Overview** (note that these two methods could easily be combined into the timer callback):

`lidar_callback` converts LIDAR data into a camera-coordinate array of points.

`camera_callback` converts these points into an array of pixels which can be viewed alongside the image, then creates the visuals with cv2. The purple dots are colored depending on how distant the point is.
