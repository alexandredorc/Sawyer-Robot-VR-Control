#!/usr/bin/env python

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from geometry_msgs.msg import Pose
from razer_hydra.msg import Hydra


class PID:
	def __init__(self,kp,ki,kd):
		self.kp=kp
		self.ki=ki
		self.kd=kd

		self.prev_error=0.0
		self.cp=0.0
		self.ci=0.0
		self.cd=0.0
		self.current_time=0.0
		self.prev_time=0.0
		self.initialize()

	def initialize(self):
		self.current_time=rospy.get_time()	
		self.prev_time=self.current_time
		self.prev_error=0.0
		self.cp=0.0
		self.ci=0.0
		self.cd=0.0

	def control_signal(self,current_state,goal_state):
		error=current_state-goal_state
		self.current_time=rospy.get_time()
		dt=self.current_time-self.prev_time
		de=error - self.prev_error
		self.cp=error
		self.ci+=error+dt
		self.cd=0
		if dt>0:
			self.cd=de/dt
		self.prev_time=self.current_time
		self.prev_error=self.error

		return ((self.kp*self.cp)+(self.ki*self.ci)+(self.kd*self.cd))

class Control:
	def __init__(self):
		self.pid=PID(0.5,0.1,0.05)
		self.limb= intera_interface.Limb('right')
		self.limb.set_joint_position_speed(0.2)
		self.go_home()
		self.gripper= intera_interface.Gripper()

	def get_current_joint(self):
		current=self.limb.endpoint_pose()
		pose=Pose()
		pose.position=current['position']
		pose.orientation=current['orientation']
		return self.IK(pose)

	def go_home(self):
		self.limb.move_to_neutral()

	def go_pose(self,pose):
		joints_state=self.IK(pose)
		self.limb.move_to_joint_position(joints_state)
	
	def IK(self,pose):
		return self.limb.ik_request(pose,'right_hand')
	
	def FK(self,joints_state):
		return self.limb.fk_request(joints_state,'right_hand')
	
	def control_arm(self,goal):
		goal_joints=self.IK(goal)
		current_joints=self.get_current_joint()
		control_signal=self.pid.control_signal(current_joints,goal_joints)
		ctrl=self.cap_joint_velocities(control_signal)
		self.set_velocities(ctrl)
		

	def cap_joint_velocities(self,ctrl):
		range_vel=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]
		for i in range(len(ctrl)):
			if range_vel<ctrl:
				ctrl[i]=range_vel
			elif -range_vel>ctrl:
				ctrl[i]=-range_vel
		return ctrl
	
	def set_velocities(self, velocities):
		joint_command = dict()
		joint_names = self.limb.joint_names()

		for i, name in enumerate(joint_names):
			joint_command[name] = velocities[i]

		self.limb.set_joint_velocities(joint_command)


class HydraCtrl:
	def __init__(self):
		self.ctrl=Control()
			
		self.xyz = None

		self.rotation = None

		self.joy = [0, 0]

		self.buttons = [0, 0, 0, 0, 0, 0, 0]

		self.trigger = 0

		self.state = False

		self.scale = 0.5

		self.key = True

		sub = rospy.Subscriber("hydra_calib", Hydra, self.hydra_callback)

	def hydra_callback(self,msg):
		paddle = msg.paddles[0]
		translation = paddle.transform.translation
		rotation = paddle.transform.rotation
		self.goal=Pose()
		self.goal.position.x=translation.x
		self.goal.position.y=translation.y
		self.goal.position.z=translation.z
		self.goal.orientation.x=rotation.x
		self.goal.orientation.y=rotation.y
		self.goal.orientation.z=rotation.z
		self.goal.orientation.w=rotation.w

		self.buttons = paddle.buttons
		self.joy = paddle.joy
		self.trigger = paddle.trigger

		self.control_arm()
		self.control_gripper()
		
	def hydra_control(self):
		self.ctrl.control_arm()
		

	def control_gripper(self):
		pass

def main():
	rospy.init_node('control_node')

	hydra = HydraCtrl()

	rs=intera_interface.RobotEnable(CHECK_VERSION)
	sawyer=Control()
	rs.enable


        
if __name__ == '__main__':
    main()