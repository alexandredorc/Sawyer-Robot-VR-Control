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
group.set_max_velocity_scaling_factor(0.4)
group.set_max_acceleration_scaling_factor(0.8)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)


# Set the planner ID to KPIECE for better singularity avoidance
#group.set_planner_id("KPIECE")

# Set the goal tolerance to 1 cm to avoid collision and singularities
group.set_goal_tolerance(0.05)

# list of singularities for each type of them
singularities = [
    [0, 0, 0, 0, 0, 0, 0],  # Wrist flip singularity
    [-0.2, -0.2,-0.2,-0.2,-0.2,-0.2,-0.2],  # Shoulder flip singularity
    """[0.1, -1.2, 0.2, 0, 1.57, 0.5, 0],  # Elbow flip singularity
    [0.5, 0, 0.5, 0, 0, 0, 0],  # Axis 1 singularity
    [0.5, -0.7, 0, 0, 0, 0, 0],  # Axis 3 singularity
    [0.5, -1.5, 0.5, 0, 0, 0, 0],  # Axis 5 singularity"""
]


# loop though all singularities to test if it avoid
count=0
for singularity in singularities:
	robot_trajectory=moveit_commander.RobotTrajectory()
	group.set_joint_value_target(singularity)
	plan = group.plan()
	if count==0:
		robot_trajectory.append(plan,0)
	else:
		time_offset=robot_trajectory.joint_trajectory.points[-1].time_from_start
		robot_trajectory+=plan
	count+=1

group.execute(robot_trajectory)

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