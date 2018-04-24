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

In the TurtleBot implementation, feature mapping is done in a very simplified manner. 

# Template Matching (Feature Recovery)

# TurtleBot Results

# Multirotor UAV Implementation

