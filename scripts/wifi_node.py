#!/usr/bin/python

import sys
import rospy

from pa2.wifi_strength import WifiStrength

if __name__ == "__main__":
    """
    Initialize node that publishes wifi strength
    """

    rospy.init_node("wifi_node", anonymous=False)

    wi_strength = WifiStrength()
    wi_strength.spin()
