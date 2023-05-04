#!/usr/bin/env python
"""Turtlebot Follower Node."""

import rospy
import math
import numpy as np
from typing import Tuple
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Follower():
    def __init__(
        self, 
        rate: int = 50,
        speed: float = 10,
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
        self.dist_right = None
        self.dist_theta = None
        
        # Initial Odometry values
        self.x_location = None
        self.y_location = None
        self.theta = None

        # Robot states
        self.com = Twist() # current command
        self.avoiding_obstacle = False # Whether we're currently trying to avoid an obstacle
        self.aligning_theta = False # Whether we're currently aligning with the theta_goal
        self.theta_ambiguity = False # Flag for when theta and theta_goal are within tolerance, but have a diference of 2pi (ie, -pi and pi)
    

    def main_control_loop(self):
        """Run the follower routine."""
        print("Running follower algorithm...")
        rospy.init_node("follower", anonymous=True)

        # Publisher object that decides what kind of topic to publish and how fast.
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # Subscribe to the Laser Scanner to get the range to the nearest obstacle
        rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
        # Subscribe to the Odometry to get robot location
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # The main loop will run at a rate in Hz
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # Here's where we publish the current commands.
            cmd_vel_pub.publish(self.com)
            
            # Sleep for as long as needed to achieve the loop rate.
            rate.sleep()

if __name__ == "__main__":
    Follower().main_control_loop()