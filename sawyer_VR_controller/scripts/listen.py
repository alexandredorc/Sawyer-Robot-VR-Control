#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

def callback_joints(msg):
    pos = msg.position 
    vel = msg.velocity
    
    rospy.loginfo(pos)
    state_msg = Float32MultiArray() 
    state_msg.data = pos + vel
    pub.publish(state_msg)
if __name__ == '__main__':
    rospy.init_node('listner_node')

    sub = rospy.Subscriber('robot/joint_states', JointState, callback_joints)

    pub = rospy.Publisher('state_listner', Float32MultiArray, queue_size=1)

    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        loop_rate.sleep()


#/robot/joint_states
