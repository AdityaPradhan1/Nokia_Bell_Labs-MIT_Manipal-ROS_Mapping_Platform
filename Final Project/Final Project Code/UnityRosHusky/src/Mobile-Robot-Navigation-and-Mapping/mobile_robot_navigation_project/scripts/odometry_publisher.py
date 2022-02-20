#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions
import tf 

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


def publish_odom_frame(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    quaternions = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    rpy = tf.transformations.euler_from_quaternion(quaternions)
    roll = rpy[0]
    pitch = rpy[1]
    yaw= rpy[2]

    quaternion = tf.transformations.quaternion_from_euler((roll), pitch, (yaw))

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.position.x
    t.transform.translation.y = msg.position.y
    t.transform.translation.z = msg.position.z
    
    t.transform.rotation.x = quaternion[0]
    t.transform.rotation.y = quaternion[1]
    t.transform.rotation.z = quaternion[2]
    t.transform.rotation.w = quaternion[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('odometry_frame_publisher')
    rospy.Subscriber('odometry_frame', geometry_msgs.msg.Pose, publish_odom_frame)
    rospy.spin()
