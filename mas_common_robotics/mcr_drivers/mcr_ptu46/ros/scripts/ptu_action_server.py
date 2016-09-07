#!/usr/bin/env python
import roslib 
#roslib.load_manifest('mcr_ptu46')
import rospy
import mcr_ptu46.msg
import actionlib
from sensor_msgs.msg import JointState
import threading
import numpy as np

class PTUControl(object):
	pan      = 0
	tilt     = 0
	pan_vel  = 0
	tilt_vel = 0
	state_lock = threading.Lock()

	def __init__(self):
		# setup some parameters
		self.tilt_speed = rospy.get_param('/mcr_ptu46/tilt_speed', 1.0)
		self.pan_speed = rospy.get_param('/mcr_ptu46/pan_speed' , 1.0)
		self.pan_step = rospy.get_param('/mcr_ptu46/pan_step', 0.00089759763795882463)
		self.tilt_step = rospy.get_param('/mcr_ptu46/tilt_step', 0.00089759763795882463)

		# setup the subscribers and publishers
		rospy.Subscriber('state', JointState, self.cb_ptu_state)
		self.ptu_pub = rospy.Publisher('cmd', JointState)
		self.as_goto_position = actionlib.SimpleActionServer('SetPTUPosition', mcr_ptu46.msg.PtuGotoPositionAction, execute_cb=self.cb_goto_position, auto_start=False)
		self.as_goto_position_by_name = actionlib.SimpleActionServer('SetPTUPositionByName', mcr_ptu46.msg.PtuGotoPositionByNameAction, execute_cb=self.cb_goto_position_by_name, auto_start=False)
		self.as_reset  = actionlib.SimpleActionServer('ResetPtu', mcr_ptu46.msg.PtuResetAction, execute_cb=self.cb_reset, auto_start=False)

		self.as_goto_position.start()
		self.as_goto_position_by_name.start()
		self.as_reset.start()

	def cb_goto_position(self, msg):
		pan, tilt, pan_vel, tilt_vel = (msg.pan, msg.tilt, msg.pan_vel, msg.tilt_vel)

		if not pan_vel:
			pan_vel = self.pan_speed
		if not tilt_vel:
			tilt_vel = self.tilt_speed

		print pan, tilt, pan_vel, tilt_vel

		self._goto(pan, tilt, pan_vel, tilt_vel)

		result = mcr_ptu46.msg.PtuGotoPositionResult()
		result.state.position = self._get_state()
		self.as_goto_position.set_succeeded(result)
		
	def cb_goto_position_by_name(self, msg):
		position = rospy.get_param('/script_server/head/' + msg.position_name)
		
		self._goto(position[0][0], position[0][1], self.pan_speed, self.tilt_speed)
		result = mcr_ptu46.msg.PtuGotoPositionByNameResult()
		result.state.position = self._get_state()
		self.as_goto_position_by_name.set_succeeded(result)
		
	def cb_reset(self, msg):
		self._goto(0, 0, self.pan_speed, self.tilt_speed)
		result = mcr_ptu46.msg.PtuResetResult()
		self.as_reset.set_succeeded(result)

	def _goto(self, pan, tilt, pan_vel, tilt_vel):
		rospy.loginfo('going to (%s, %s) with (%s, %s)' % (pan, tilt, self.pan_speed, self.tilt_speed))
		msg_out = JointState()
		msg_out.header.stamp = rospy.Time.now()
		msg_out.name = ['pan', 'tilt']
		msg_out.position = [pan, tilt]
		msg_out.velocity = [pan_vel, tilt_vel]
		self.ptu_pub.publish(msg_out)
		# wait for it to get there
		wait_rate = rospy.Rate(10)
		while not self._at_goal((pan, tilt)) and not rospy.is_shutdown():
			wait_rate.sleep()

	def _at_goal(self, goal):
		return all(np.abs(np.array(goal) - (self.pan, self.tilt)) <= (self.pan_step, self.tilt_step))

	def cb_ptu_state(self, msg):
		self.state_lock.acquire()
		
		for i in range(len(msg.name)):
			if msg.name[i] == 'ptu_tilt_joint':
				self.tilt = msg.position[i]
			elif msg.name[i] == 'ptu_pan_joint':
				self.pan = msg.position[i]
				
		self.state_lock.release()


	def _get_state(self):
		self.state_lock.acquire()
		pt = (self.pan, self.tilt)
		self.state_lock.release()
		return pt

if __name__ == '__main__':
	rospy.init_node('mcr_ptu46_action_server')
	PTUControl()
	rospy.spin()
