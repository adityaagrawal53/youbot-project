#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist 

import time, math
rospy.init_node('simple_square_movement', anonymous=True)
def move_fwd(): 

	
	#Publisher to /cmd_vel
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	
	#Create a Twist message for movement
	move_cmd = Twist()
	move_cmd.linear.x = 0.2 # Move at 0.1 m/s
	
	#Set distance and calculate time needed
	distance = 1 
	speed = move_cmd.linear.x 
	move_time = distance / speed 
	
	#Set a rate to publish messages 

	start_time = time.time()
	
	while time.time() - start_time < move_time and not rospy.is_shutdown():
		pub.publish(move_cmd)


	#After 10 cm, stop robot 
	move_cmd.linear.x = 0 
	pub.publish(move_cmd)

	rospy.loginfo("Moved forward 10cm.")
	rospy.sleep(2)

def turn_right(): 
	
	
	#Publisher to /cmd_vel
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	
	#Create a Twist message for movement
	move_cmd = Twist()
	move_cmd.angular.z = -math.pi/5 # Move at 0.1 m/s
	rospy.loginfo(math.pi)	


	#Set distance and calculate time needed
	distance = math.pi*89 / 180
	speed = move_cmd.angular.z
	move_time = -distance / speed 
	
	#Set a rate to publish messages 

	start_time = time.time()
	
	while time.time() - start_time < move_time and not rospy.is_shutdown():
		pub.publish(move_cmd)


	#After 10 cm, stop robot 
	move_cmd.angular.z = 0 
	pub.publish(move_cmd)

	rospy.loginfo("Turned 90 degrees clockwise.")
	rospy.sleep(2)


if __name__ == '__main__':
	try:
		move_fwd()
		turn_right()
		move_fwd()
		turn_right()
		move_fwd()
		turn_right()
		move_fwd()
		turn_right()
	except rospy.ROSInterruptException:
		pass
