#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget


def callback(velocity):

    pub_vel = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    twist = PositionTarget()
    twist.header.frame_id = 'world'
    twist.coordinate_frame = 8
    twist.type_mask = 1479
    twist.velocity.x = velocity.linear.x
    twist.velocity.y = velocity.linear.y
    twist.yaw_rate = velocity.angular.z
    
    rospy.loginfo(twist)
    pub_vel.publish(twist)  

def cmd_vel_to_mavros():
    
    rospy.init_node('cmd_vel_to_mavros', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    cmd_vel_to_mavros()