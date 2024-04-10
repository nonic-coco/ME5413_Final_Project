#!/usr/bin/env python
import rospy
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

# Initialize the ROS node
rospy.init_node('robot_vision_and_movement', anonymous=True)

# Publishers and subscribers
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
image_sub = rospy.Subscriber("/camera/image_raw", Image, queue_size=1, callback=None)  # We'll set the callback later

# Global variable to keep track of image processing
detected = False
template = cv2.imread('number2.png', 0)  # Make sure the template image is in the same directory as your script
w, h = template.shape[::-1]

# Function to rotate the robot
def rotate(degrees_per_second, duration_in_seconds=None):
    global detected

    radians_per_second = math.radians(degrees_per_second)
    twist = Twist()
    twist.angular.z = radians_per_second

    rate = rospy.Rate(10)  # 10 Hz

    # If duration is None, rotate indefinitely
    if duration_in_seconds is not None:
        end_time = rospy.Time.now() + rospy.Duration(duration_in_seconds)
    else:
        end_time = None

    while not rospy.is_shutdown() and (end_time is None or rospy.Time.now() < end_time) and not detected:
        cmd_vel_pub.publish(twist)
        rate.sleep()

    # Stop the robot
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)

# Image processing callback function
def image_callback(msg):
    global detected, bridge, cmd_vel_pub, template, w, h

    if detected:
        # If we've already detected the '2', we don't need to process any more images
        return

    try:
        # Convert ROS Image to OpenCV Image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(gray_image, template, cv2.TM_CCOEFF_NORMED)
    threshold = 0.8
    loc = np.where(res >= threshold)

    for pt in zip(*loc[::-1]):  # If there is at least one match
        detected = True
        break

# The main function to control the robot
def main():
    global detected, bridge, image_sub

    bridge = CvBridge()

    # Check for the '2' before starting to rotate
    rospy.sleep(1)  # Wait for the first image to arrive
    if not detected:
        # If not detected, start rotating
        image_sub.callback = image_callback  # Set the callback for image processing
        rotate(30)  # Rotate indefinitely at 30 degrees per second

    # If detected, stop and move towards the '2'
    if detected:
        # Align with the '2' and move forward (this part you would need to fill in with your logic)
        # For example, to move forward:
        move = Twist()
        move.linear.x = 0.1  # Move forward at 0.1 m/s
        cmd_vel_pub.publish(move)
        rospy.sleep(3)  # Move forward for 3 seconds
        move.linear.x = 0  # Stop moving
        cmd_vel_pub.publish(move)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
