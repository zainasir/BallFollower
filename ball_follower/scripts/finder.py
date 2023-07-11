#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters

# Camera intrinsic values
FOCAL_X = 570.3405082258201
FOCAL_Y = 570.3405082258201
PRINCIPAL_X = 319.5
PRINCIPAL_Y = 239.5

# Linear Distancing values
TARGET_DISTANCE = 1000

# Initialize the CvBridge class
bridge = CvBridge()

# Initialize the velocity publisher
vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

def image_callback(msg, depth_msg):
    # Convert the ROS message to a CV image and depth image
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

    # Convert the image to the HSV color space
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define the range of red color in HSV
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv_image, lower_red, upper_red)

    lower_red = np.array([170, 100, 100])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv_image, lower_red, upper_red)

    # Combine the two masks
    mask = cv2.bitwise_or(mask1, mask2)

    # Find the contours in the mask
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        (x,y), radius = cv2.minEnclosingCircle(largest_contour)
        center = (int(x),int(y))
        radius = int(radius)

        # Draw a circle around the ball
        cv2.circle(cv_image, center, radius, (0, 255, 0), 2)

        # Compute the error between the center of the image and the center of the ball
        image_center = cv_image.shape[1] / 2
        ball_error = center[0] - image_center

        # Compute the distance to the ball
        distance = depth_image[int(y), int(x)]

        # Compute the linear and angular velocities
        if distance > 0 and abs(TARGET_DISTANCE - distance) > 25:
            if distance > TARGET_DISTANCE:
                linear_velocity = 0.001 * abs(TARGET_DISTANCE - distance)
            else:
                linear_velocity = -0.001 * abs(TARGET_DISTANCE - distance)
        else:
            linear_velocity = 0.0

        angular_velocity = -0.01 * ball_error

        # Publish the velocity command
        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        vel_pub.publish(vel_msg)

    # Display the image
    cv2.imshow('image', cv_image)
    cv2.waitKey(3)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('ball_follower')

    # Subscribe to the camera image topic and depth topic
    rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)

    # Synchronize both images
    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
    ts.registerCallback(image_callback)

    # Spin until Ctrl-C is pressed
    rospy.spin()

    # Clean up the OpenCV windows
    cv2.destroyAllWindows()
