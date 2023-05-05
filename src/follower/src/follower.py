#!/usr/bin/env python
"""Turtlebot Follower Node."""

import rospy
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import cv2

class Follower():
    def __init__(
        self, 
        rate: int = 50,
        speed: float = 1,
        spin_speed: float = 2,
        min_dist: float = 3,
    ):
        """Initialize the following algorithm params."""
        self.rate = rate
        self.speed = speed 
        self.spin_speed = spin_speed
        self.min_dist = min_dist
        
        # Initial LaserScan values
        self.dist_ahead = None
        
        # Initial Odometry values
        self.x_location = None
        self.y_location = None

        # Initial Image
        self.image = None

        # Robot states
        self.com = Twist() # current command


    def image_callback(self, msg: Image):
        """Process the image data."""
        self.image = msg.data
    

    def main_control_loop(self):
        """Run the follower routine."""
        print("Running follower algorithm...")
        rospy.init_node("follower", anonymous=True)

        # Publisher object that decides what kind of topic to publish and how fast.
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # Subscribe to the Camera to get the image data
        rospy.Subscriber("/image_raw", Image, self.image_callback)
        
        # Subscribe to the Laser Scanner to get the range to the nearest obstacle
        # rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
        
        # # Subscribe to the Odometry to get robot location
        # rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # The main loop will run at a rate in Hz
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # Here's where we publish the current commands.
            cmd_vel_pub.publish(self.com)
            # Show the image
            cv2.imshow("Image", self.image)
            # Sleep for as long as needed to achieve the loop rate.
            rate.sleep()


if __name__ == "__main__":
    Follower().main_control_loop()