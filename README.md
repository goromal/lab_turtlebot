Feature Mapping and Recovery with Template Matching
===================

This is an example of a Robotic Vision class project using the TurtleBot. If you are unfamiliar with the TurtleBot, please start with the RoboticVision branch of this repository and the instructions found in the wiki.

This package is a good example of several additional features that can be useful in other TurtleBot project including:
- Interfacing with the Motion Capture system in the MAGICC Lab
- Writing a ROS node in C++ that uses the TurtleBot's images

A short description of the methods used is included here along with a videos showing results on the TurtleBot and on a multirotor UAV in a Gazebo simulation. Development is still underway to fly these algorithms on a multirotor in hardware.

# Feature Detection and Tracking

In the implementation included here, up to 20 features are detected and tracked. The OpenCV function `goodFeaturesToTrack()` is used to detect corners. Found [here](https://github.com/goromal/lab_turtlebot/blob/6fd1b4d7ee1b72b1b18bc376c53c36fa14aa6ece/map_turtle/src/feature_tracker/feature_tracker.cpp#L31) in the code, the feature detector imposes strict requirments so that corners detected are high quality and spaced apart from one another.

While features remain in the camera image, OpenCV's `calcOpticalFlowPyrLK()` is used to track them as seen [here](https://github.com/goromal/lab_turtlebot/blob/6fd1b4d7ee1b72b1b18bc376c53c36fa14aa6ece/map_turtle/src/feature_tracker/feature_tracker.cpp#L39) in the code. When optical flow fails to track a feature, the features is removed from the vector of currently tracked features. Features are also removed if they get closer to the edge of the image than the pixel size of the feature template (described more in the Template Matching section).

# Feature Mapping

In the TurtleBot implementation, feature mapping is done in a very simplified manner. This takes place primarily in the `Estimator()` class found [here](https://github.com/goromal/lab_turtlebot/blob/7ba396c1c5c26ee2df5d86803d0776e675611d28/map_turtle/src/feature_tracker/estimator.cpp#L8). Feature locations are estimated by estimating the global azimuth and elevation angle to each feature. Since the TurtleBot only spins in a circle in the middle of the room, the angles to each feature should not change with time.

Each time a feature is detected in an image, the `update()` function is called with the feature id, pixel location and the current global yaw angle of the TurtleBot (the yaw angle of the TurtleBot is updated at about 100 Hz from the motion capture system). This `update()` function invokes a simple low pass filter on the estimates.

# Template Matching (Feature Recovery)

The main part of this project is the feature recovery done by template matching. Template matching is a technique where you scan a smaller, template image across another image and score how well the pixels match at each location. A threshold must then be used to know if the best matched location is a true match with the template image or not. Template matching has many negatives about it including being computationally heavy and not being invariant to rotation or scale. However, template matching is pretty good at finding the matching location for a template image.

As the TurtleBot spins in a circle, an estimated pixel located is calculated for each feature that is not currently being tracked in the image. Once the estimated pixel location is in the image, a search region is created around the estimated location. The template image for the feature is then scanned across the search region and the score for the best matched location is compared to a threshold value that was found empirically. This approach is depicted in the following image.

![Template Matching](https://raw.githubusercontent.com/goromal/lab_turtlebot/mocap_map/images/template_match.png)

# TurtleBot Results

These algorithms combine to yield the results seen in the video here.

[![UAV TurtleBot Results](https://img.youtube.com/vi/qZHhF-QcZOQ/0.jpg)](https://www.youtube.com/watch?v=qZHhF-QcZOQ)


# Multirotor UAV Implementation

A similiar implementation is being worked on for a multirotor UAV. Feature detection, tracking and recovery happen identically to the methods described above, however, feature mapping (locating their position in the world) happens in a more sophisticated fashion. The estimation follows the work from Jeff Ferrin's [Autonomous Goal-Based Mapping and Navigation Using a Ground Robot](https://scholarsarchive.byu.edu/etd/6190/). The current progress can be seen in the video included here. Though work is not finished, the feature detection, tracking and recovery seem to work well. The remaining work revolves around the estimation. However, the algorithms are expected to fly in hardware in Jun 2018. 

[![UAV Simulation Results](https://img.youtube.com/vi/2LMUWCHhFOE/0.jpg)](https://www.youtube.com/watch?v=2LMUWCHhFOE)

