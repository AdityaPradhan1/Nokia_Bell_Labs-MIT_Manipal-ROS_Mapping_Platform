#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


def clbk_laser(msg):
    # 720/5 = 144
    # 270/5 = 54
    regions = [
        min(min(msg.ranges[0:53]), 10),
        min(min(msg.ranges[54:107]), 10),
        min(min(msg.ranges[108:161]), 10),
        min(min(msg.ranges[162:215]), 10),
        min(min(msg.ranges[216:270]), 10),
    ]
    rospy.loginfo(regions)


def main():
    rospy.init_node('reading_laser')
    sub = rospy.Subscriber("/laser_scan", LaserScan, clbk_laser)

    rospy.spin()


if __name__ == '__main__':
    main()
