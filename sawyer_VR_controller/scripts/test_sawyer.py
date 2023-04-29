#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Initialize ROS node and MoveIt commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("right_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)


# Set the planner ID to KPIECE for better singularity avoidance
group.set_planner_id("KPIECE")

# Set the goal tolerance to 1 cm to avoid collision and singularities
group.set_goal_tolerance(0.01)

# list of singularities for each type of them
singularities = [
    [0, 0, 0, 0, 0, 0, 0],  # Wrist flip singularity
    [0.1, -1.0, 0.2, 0, -1.57, 0.5, 0],  # Shoulder flip singularity
    [0.1, -1.2, 0.2, 0, 1.57, 0.5, 0],  # Elbow flip singularity
    [0.5, 0, 0.5, 0, 0, 0, 0],  # Axis 1 singularity
    [0.5, -0.7, 0, 0, 0, 0, 0],  # Axis 3 singularity
    [0.5, -1.5, 0.5, 0, 0, 0, 0],  # Axis 5 singularity
]

# loop though all singularities to test if it avoid
for singularity in singularities:
    group.set_joint_value_target(singularity)
    plan = group.plan()
    group.go(wait=True)

# Define the target position in 3D space
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.0
target_pose.position.z = 0.5
group.set_pose_target(target_pose)

# Define and execute the plan to reach the target position
plan = group.plan()
group.execute(plan, wait=True)
rospy.sleep(1)

# Shutdown the MoveIt commander
moveit_commander.roscpp_shutdown()