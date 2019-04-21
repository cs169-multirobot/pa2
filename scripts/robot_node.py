#!/usr/bin/python

import sys
import rospy

from pa2.robot import Robot

if __name__ == "__main__":
    """
    Initialize node that controls the turtlebot
    """

    rospy.init_node("robot_follower", anonymous=False)

    bot = Robot()
    bot.spin()
