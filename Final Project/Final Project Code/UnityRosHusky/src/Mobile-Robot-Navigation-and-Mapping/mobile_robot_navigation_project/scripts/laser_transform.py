#!/usr/bin/env python  

import rospy

# Because of transformations
import tf_conversions
import tf 

import tf2_ros
import geometry_msgs.msg


def publish_laser_frame(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    roll = 3.14159
    pitch = 0
    yaw= 0

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "base_laser"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    
    t.transform.rotation.x = quaternion[0]
    t.transform.rotation.y = quaternion[1]
    t.transform.rotation.z = quaternion[2]
    t.transform.rotation.w = quaternion[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('laser_transform_publisher')
    rospy.Subscriber('odometry_frame', geometry_msgs.msg.Pose, publish_laser_frame)
    rospy.spin()