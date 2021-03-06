import rospy
import math
from pa2.robot_math import *

# different plans for turtlebot
WIFI_FOLLOW = "wifi follow"
OBST_AVOID = "obstacle avoidance"

# WIFI_FOLLOW states
COLD = "COLD"       # turtlebot doesn't know where the wifi source is
WARM = "WARM"       # turtlebot is waiting for the right moment to head towards the wifi source
HOT = "HOT"         # turtlebot is turning towards the wifi source
BURNING = "BURN" # turtlebot is heading towards the wifi source

# OBST_AVOID states
OBSTACLE = "OBST"   # turtlebot is avoiding obstacle
FOLLOW = "FOLLOW"       # turtlebot is waiting for there to be no obstacles in its view


class State():
    def __init__(self):
        self.state = BURNING
        self.plan = WIFI_FOLLOW

	self.cur_odom = 0.0
	self.target_odom = 0.0


    def reinitialize(self):
        """
        Reinitialize the robot.
        """
        self.state = BURNING
        self.plan = WIFI_FOLLOW


    def wifi_check(self, cur_strength, prev_strength):
        """
        Depending on the current state and wifi strength, the state of the
        turtlebot may change to help it move towards the wifi source.

        Args:
            cur_strength: Current wifi strength
            prev_strength: Previous wifi strength
            cur_odom: Current yaw of the turtlebot

        Returns:
            target_odom: Target yaw for the turtlebot to head if it is in the HOT state
        """

        if self.plan == WIFI_FOLLOW:
            # if no idea where the wifi source is, wait for an increase in wifi strength
            if self.state == COLD:
                if cur_strength > prev_strength:
                    self.state = WARM

            # waiting for the peak wifi strength
            elif self.state == WARM:
                if cur_strength < prev_strength:
                    self.state = HOT
                    self.target_odom = self.cur_odom + math.pi
                    self.target_odom = rectify_angle_pi(self.target_odom)

            # waiting to see if reheading is needed
            elif self.state == BURNING:
                if cur_strength < prev_strength:
                    self.state = COLD


    def odom_check(self, cur_odom):
        """
        Depending on the current state, the odometry values might be needed to
        check if the turtlebot needs to stop turning.

        Args:
            cur_odom: Current yaw of the turtlebot
            target_odom: Target yaw of the turtlebot
        """

	self.cur_odom = cur_odom

        # if the turtlebot is turning towards the wifi source, check if it needs to stop turning
        if self.state == HOT or self.state == FOLLOW:
            if (self.cur_odom > 0.0 and self.target_odom < 0.0) or (self.cur_odom < 0.0 and self.target_odom > 0.0):
                error = abs(rectify_angle_2pi(self.cur_odom) - rectify_angle_2pi(self.target_odom))
            else:
                error = abs(self.cur_odom - self.target_odom)

            if error < 0.2:
		self.plan = WIFI_FOLLOW
                self.state = BURNING


    def scan_check(self, is_obstacle):
        """
        Depending on the current state and if there is an obstacle in the view,
        change the state to help the turtlebot avoid collision.

        Args:
            is_obstacle: Boolean that indicates the presence of an obstacle
        """
        if is_obstacle:
            # new obstacle
            if self.plan == WIFI_FOLLOW:
                    self.state = OBSTACLE
                    self.plan = OBST_AVOID
            # already knew there was an obstacle
            else:
                self.state = OBSTACLE
        # if obstacle has been avoided
        else:
            if self.plan == OBST_AVOID:
		if self.state != FOLLOW:
		    self.state = FOLLOW
		    self.target_odom = rectify_angle_pi(self.cur_odom + math.pi / 2)




    def move(self, LINEAR_SPEED, ANGULAR_SPEED, NO_SPEED):
        """
        Depending on the current state, decide on the robot movement parameters.

        Args:
            LINEAR_SPEED: static value for the linear speed of the turtlebot
            ANGULAR_SPEED: static value for the angular speed of the turtlebot
            NO_SPEED: 0.0
        Returns:
            linear_x and angular_z: Linear and angular speed for the behavior to be implemented
        """

        if self.state == COLD or self.state == WARM:
            linear_x = LINEAR_SPEED
            angular_z = ANGULAR_SPEED

        elif self.state == HOT:
            linear_x = NO_SPEED
            angular_z = -ANGULAR_SPEED

        elif self.state == BURNING:
            linear_x = LINEAR_SPEED
            angular_z = NO_SPEED

        elif self.state == OBSTACLE:
            linear_x = NO_SPEED
            angular_z = -ANGULAR_SPEED

        elif self.state == FOLLOW:
            linear_x = LINEAR_SPEED
            angular_z = ANGULAR_SPEED

        return linear_x, angular_z
