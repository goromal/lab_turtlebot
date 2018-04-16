#!/usr/bin/env python
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class CVControl:

    def __init__(self):
        # Turtlebot command publisher
        self.cmd_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        self.cmd = Twist()

        # Image subscriber
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/decompressed_img", Image, self.img_callback)

    def img_callback(self, data):
        # Convert ROS msg image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # Draw circle on image
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        # Display OpenCV Image
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # Send Velocity command to turtlebot
        v = 0.0
        w = 0.5
        self.send_command(v, w)

    def send_command(self, v, w):
        # Put v, w commands into Twist message
        self.cmd.linear.x = v
        self.cmd.angular.z = w

        # Publish Twist command
        self.cmd_pub.publish(self.cmd)

def main():

    # Initialize ROS node
    rospy.init_node('image_converter')

    # Instantiate control class
    ctrl = CVControl()

    # Keep running until Ctrl-C to kill
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
