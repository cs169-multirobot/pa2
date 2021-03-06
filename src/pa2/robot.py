import rospy
import numpy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from pa2.msg import Wifi

from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

from pa2.robot_math import *
from pa2.state import State

MIN_SCAN = 0.05
SAFE_DISTANCE = 0.25
VIEW_RANGE = math.pi / 4
LINEAR_SPEED = 0.07
ANGULAR_SPEED = 0.3
NO_SPEED = 0.0

class Robot():
    def __init__(self):

        # information related to the wifi readings
        self.target_strength = rospy.get_param("target_strength")
        self.cur_strength = 0
        self.prev_strength = 1

        # information related to the odometry of the turtlebot
        self.odom = None
        self.target_odom = None
        self.is_obstacle = False

        # information related to the state of the turtlebot and if it is moving
        self.state = State()
        self.is_moving = True

        # subscribers that help with the behaviors of the turtlebot
        rospy.Subscriber("wifi_strength", Wifi, self.wifi_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)

        # publisher to move the turtlebot
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # service to handle the turtlebot stopping and restarting
        self.safe_stop = rospy.Service("safe_start_stop", Trigger, self.handle_start_stop)


    def wifi_callback(self, msg):
        """
        Callback for retrieving and handling the wifi strength.

        Args:
            msg: Wifi message that has the current wifi strength reading.
        """
        self.wifi_strength = msg.signal_strength

	if self.odom != None:
            # if the turtlebot is in the wifi target range, then stop
            if abs(self.wifi_strength - self.target_strength) < 1.0:
                self.is_moving = False

	        r = rospy.Rate(2)
	        cmd_msg = Twist()
	        cmd_msg.linear.x = NO_SPEED
	        cmd_msg.angular.z = NO_SPEED
	        self.cmd_pub.publish(cmd_msg)

	        r.sleep()

            # otherwise, check what the new state should be according to the new reading
            else:
	        self.state.odom_check(self.odom)
	        self.state.scan_check(self.is_obstacle)
                self.target_odom = self.state.wifi_check(self.wifi_strength, self.prev_strength)

            self.prev_strength = self.wifi_strength



    def odom_callback(self, msg):
        """
        Callback for retrieving and handling the robot's position and orientation.

        Args:
            msg: Odometry msg that has the current robot's position and orientation.
        """
        self.odom = quat_to_euler(msg.pose.pose.orientation)
        self.odom = rectify_angle_pi(self.odom)


    def scan_callback(self, msg):
        """
        Callback for retrieving and handling the robot's laser scan readings.

        Args:
            msg: LaserScan msg describing the robot's laser scan readings.
        """
        ranges = numpy.array(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # find the closest obstacle in viewing range
        min_reading = msg.range_max
        for i in range(len(ranges)):
            if -VIEW_RANGE < angle_min + angle_inc * i < VIEW_RANGE:
                if MIN_SCAN < ranges[i] < min_reading:
                    min_reading = ranges[i]

        # is the obstacle within the turtlebot's safety area
        self.is_obstacle = False
        if min_reading < SAFE_DISTANCE:
            self.is_obstacle = True


    def handle_start_stop(self, req):
        """
        Callback for a request to stop or start the turtlebot.

        Args:
            msg: Trigger msg.
        """

        # if turtlebot is moving, stop it
        if self.is_moving:
            self.is_moving = False

	    r = rospy.Rate(2)

            cmd_msg = Twist()
            cmd_msg.linear.x = NO_SPEED
            cmd_msg.angular.z = NO_SPEED
            self.cmd_pub.publish(cmd_msg)

            r.sleep()

            return TriggerResponse(True, "Robot safely stopped.")

        # if turtlebot is not moving, start it
        else:
            self.is_moving = True
            self.state.reinitialize()

            return TriggerResponse(True, "Robot safely started.")


    def spin(self):
        """
        The robot should continously act in some behavior.
        """
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.is_moving:
                # get the movement parameters according to the current state
                linear_x, angular_z = self.state.move(LINEAR_SPEED, ANGULAR_SPEED, NO_SPEED)

                cmd_msg = Twist()
                cmd_msg.linear.x = linear_x
                cmd_msg.angular.z = angular_z
                self.cmd_pub.publish(cmd_msg)

                r.sleep()
