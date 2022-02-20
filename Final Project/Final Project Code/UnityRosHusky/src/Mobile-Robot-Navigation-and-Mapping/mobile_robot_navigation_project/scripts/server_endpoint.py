#!/usr/bin/env python

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import LaserScan


def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'Server')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 10000000)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)
    
    tcp_server.start({
        'cmd_vel': RosSubscriber('cmd_vel', Twist, tcp_server),
        'laser_scan': RosPublisher('laser_scan', LaserScan, queue_size = 100000),
        'odometry_frame': RosPublisher('odometry_frame', Pose, queue_size = 100000),
        'move_base_simple/goal': RosPublisher('move_base_simple/goal', PoseStamped, queue_size = 1)
    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
