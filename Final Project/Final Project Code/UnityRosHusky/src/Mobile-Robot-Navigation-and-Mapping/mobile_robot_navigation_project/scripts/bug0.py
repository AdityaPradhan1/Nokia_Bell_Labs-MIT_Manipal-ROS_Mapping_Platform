#! /usr/bin/env python

"""
.. module:: bug0
    :platform: Unix
    :synopsis: Python module for implementing the bug0 path planning algorithm
.. moduleauthor:: Omotoye Adekoya adekoyaomotoye@gmail.com 

This node implements the bug0 path planning algorithm for moving a robot from its current 
position to some target position.      

Subscribes to:
    /odom topic where the simulator publishes the robot position
    /laser_scan topic where the robot publishes the laser scan readings

Publishes to: 
    /cmd_vel publishes velocity command to this topic
    
Service:
    /go_to_point_switch sends a goal request to the go to point server 
    /wall_follower_switch sends a request to the wall follower server 
    /bug_switch accepts a request and sends a response to the bug switch client 
    
"""


import rospy
import time

# import ros message
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations

# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist
from final_assignment.srv import MoveBaseResult, MoveBaseResultResponse

import math

pub = None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0
regions_ = None
state_desc_ = ["Go to point", "wall following", "target reached"]
state_ = 0
# 0 - go to point
# 1 - wall following

# callbacks


def clbk_odom(msg):
    """
    The pose callback function that takes the position and posture of
    the robot from the argument "msg" and set it to
    two global variables containing the x, y coordinates pose and yaw angle.

    Args:
        pose_message (Odom): an object containing all the values
        of the current position and posture of the robot
    """

    global position_, yaw_

    # position
    position_ = msg.position

    # yaw
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    )
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    """A callback function that takes in the Laser scan reading from the
    argument "msg" and then interpret it by disecting it into regions


    Args:
        msg (LaserScan): an object containing the laser scan values
        coming from the laser sensor of the robot.
    """
    global regions_
    regions_ = {
        "left": min(min(msg.ranges[0:53]), 10),
        "fleft": min(min(msg.ranges[54:107]), 10),
        "front": min(min(msg.ranges[108:161]), 10),
        "fright": min(min(msg.ranges[162:215]), 10),
        "right": min(min(msg.ranges[216:270]), 10),
    }


def handle_result(mes):
    """This is a callback function that handles the request from a client service
    to start the bug0 node by changing the state to 0 which makes the node active

    Args:
        mes (string): a request message sent by the client calling the service.

    Returns:
        [MoveBaseResultResponse]: returns a response message "Target Reached" when the
        target has been reached.
    """
    global state_
    time.sleep(1)
    change_state(0)
    count = 0
    while state_ != 4 and count != 30:
        time.sleep(2)
        count += 1
    if state_ == 4:
        res = "Target Reached"
        print(res)
    elif count == 300:
        res = "Target could not be reached"
        change_state(2)
        print(res)

    return MoveBaseResultResponse(res)


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
        state_ = 4


def normalize_angle(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, pub

    rospy.init_node("bug0")

    sub_laser = rospy.Subscriber("/laser_scan", LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber("/odometry_frame", Pose, clbk_odom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    srv_client_go_to_point_ = rospy.ServiceProxy("/go_to_point_switch", SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy("/wall_follower_switch", SetBool)
    t = rospy.Service("bug_switch", MoveBaseResult, handle_result)

    # initialize going to the point
    change_state(2)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        if state_ == 4:
            continue
        if state_ == 0:
            err_pos = math.sqrt(
                pow(desired_position_.y - position_.y, 2)
                + pow(desired_position_.x - position_.x, 2)
            )
            if err_pos < 0.3:
                change_state(2)

            elif regions_["front"] < 0.5:
                change_state(1)

        elif state_ == 1:
            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x
            )
            err_yaw = normalize_angle(desired_yaw - yaw_)
            err_pos = math.sqrt(
                pow(desired_position_.y - position_.y, 2)
                + pow(desired_position_.x - position_.x, 2)
            )

            if err_pos < 0.3:
                change_state(2)
            if regions_["front"] > 1 and math.fabs(err_yaw) < 0.05:
                change_state(0)

        elif state_ == 2:
            desired_position_.x = rospy.get_param("des_pos_x")
            desired_position_.y = rospy.get_param("des_pos_y")
            err_pos = math.sqrt(
                pow(desired_position_.y - position_.y, 2)
                + pow(desired_position_.x - position_.x, 2)
            )
            if err_pos > 0.35:
                change_state(0)

        rate.sleep()


if __name__ == "__main__":
    main()
