import rospy
from pa2.msg import Wifi

NET_FILE = "/proc/net/wireless"

class WifiStrength:
    def __init__(self):

        self.signal_strength = 0

        self.pub = rospy.Publisher('wifi_strength', Wifi)


    def spin(self):
        r = rospy.Rate(10)
        msg = Wifi()

        while not rospy.is_shutdown():
            self.read_wifi_strength()

            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "wifi_line"
            msg.signal_strength = self.signal_strength
            self.pub.publish(msg)

            r.sleep()



    def read_wifi_strength(self):
        file = open(NET_FILE, "r")

        for i, line in enumerate(file):
            if i == 2:
                data = line.split()
                self.signal_strength = float(data[3])
                break

        file.close()
