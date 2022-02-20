#! /usr/bin/env python

"""
.. module:: go_to_point_service_m 
    :platform: Unix
    :synopsis: Python module for control of a mobile robot to navigate to a target point
.. moduleauthor:: Omotoye Adekoya adekoyaomotoye@gmail.com 

This node controls a mobile robot to move from it position to some target position

Subscribes to:
    /odom topic where the simulator publishes the robot position

Publishes to: 
    /cmd_vel velocity to move to the desired robot positions
    
Service:
    /go_to_point_switch accepts a request to go to a target position 
    
"""


# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose

# from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.3

kp_a = 3.0  # In ROS Noetic, it may be necessary to change the sign of this proportional controller
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publishers
pub = None

# service callbacks


def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = "Done!"
    return res


# callbacks


def clbk_odom(msg):
    global position_
    global yaw_

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


def change_state(state):
    global state_
    state_ = state
    print("State changed to [%s]" % state_)


def normalize_angle(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a * err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print("Yaw error: [%s]" % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(
        pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2)
    )

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = kp_d * (err_pos)
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a * err_yaw
        pub.publish(twist_msg)
    else:
        print("Position error: [%s]" % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print("Yaw error: [%s]" % err_yaw)
        change_state(0)


def done(des_pos):
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    err_pos = math.sqrt(
        pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2)
    )
    if err_pos > 0.35:
        change_state(0)


def main():
    global pub, active_, desired_position_

    rospy.init_node("go_to_point")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    sub_odom = rospy.Subscriber("/odometry_frame", Pose, clbk_odom)

    srv = rospy.Service("go_to_point_switch", SetBool, go_to_point_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        if not active_:
            continue
        else:
            desired_position_.x = rospy.get_param("des_pos_x")
            desired_position_.y = rospy.get_param("des_pos_y")
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done(desired_position_)
            else:
                rospy.logerr("Unknown state!")

        rate.sleep()


if __name__ == "__main__":
    main()
