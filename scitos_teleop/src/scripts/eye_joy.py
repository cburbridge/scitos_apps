#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState, Joy


class EyeJoy():
	"A class to command the Scitos head from a joystick"


	def __init__(self):
		rospy.init_node('eye_joy')
		self.pub = rospy.Publisher('/head/commanded_state', JointState)
		rospy.Subscriber("/joy", Joy, self.callback) 
		rospy.logdebug(rospy.get_name() + " setting up")
		self.command = JointState() 
		self.command.name=["EyeLidLeft", "EyeLidRight"]
		self.eye_command = JointState()
		self.eye_command.name=["EyesTilt", "EyesPan"] 

	def callback(self, joy): 
		rospy.logdebug(rospy.get_name() + ": I heard %s" % joy)		
		#update command
		self.eye_command.position=[joy.axes[4]*100, joy.axes[3]*100] 

		# publish
		self.pub.publish(self.eye_command)		
			
		print joy.axes[5] 

		#update command
		self.command.position=[(joy.axes[5]+1)*50]*2 

		# publish
		self.pub.publish(self.command)

if __name__ == '__main__':
    head_joy = EyeJoy()
    rospy.spin()
