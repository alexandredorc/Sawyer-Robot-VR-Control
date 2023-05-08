#!/usr/bin/env python

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float32MultiArray
import threading
from razer_hydra.msg import Hydra


def hydra_callback(msg):
    global target_position
    xyz = msg.paddles[1].transform.translation
    target_position=xyz


def main():
    global target_position

    # Initialize the moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('sawyer_hydra_follow_moveit')

    # Subscribe to the 'hydra_calib' topic
    rospy.Subscriber("hydra_calib", Hydra, hydra_callback)

    # Initialize the robot and MoveGroupCommander for the arm
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")
    group.set_max_velocity_scaling_factor(0.5)

    rate = rospy.Rate(20)  # 100 Hz update rate

    while not rospy.is_shutdown():
        
        if target_position is not None:
            rospy.logwarn(target_position.x)
            # Convert Hydra position to a Pose message

            # Set target pose and plan
            group.set_position_target([target_position.x,target_position.y,target_position.z])
            plan = group.plan()

            # Execute the plan
            group.execute(plan, wait=True)

            if plan.joint_trajectory.points:
                rospy.logerr("Executing plan")
                # Execute the plan
                group.execute(plan)
            else:
                rospy.logerr("Failed to find a valid plan")

        rate.sleep()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        target_position = None
        main()
    except rospy.ROSInterruptException:
        pass
