#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 
import sys

import time, math, tf
rospy.init_node('rotate_test', anonymous=True)
def move_fwd(speed, distance): 

	
	#Publisher to /cmd_vel
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	
	#Create a Twist message for movement
	move_cmd = Twist()
	move_cmd.linear.x = float(speed) # m/s
	
	#Calculate time needed
	move_time = float(distance) / float(speed) 
	
	#Set a rate to publish messages 

	start_time = time.time()
	
	while time.time() - start_time < move_time and not rospy.is_shutdown():
		pub.publish(move_cmd)


	#After specified distance, stop robot 
	move_cmd.linear.x = 0 
	pub.publish(move_cmd)

	rospy.loginfo("Moved forward " + str(int(100 * distance)) + " cm.")
	rospy.loginfo("%.2f", move_time)
	
def turn_right(speed, distance): 
	
	
	#Publisher to /cmd_vel
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	
	#Create a Twist message for movement
	move_cmd = Twist()
	move_cmd.angular.z = float(speed) # rad/s

	#Set distance and calculate time needed
	move_time = float(distance) / float(speed) 
	
	#Set a rate to publish messages 

	start_time = time.time()
	
	while time.time() - start_time < move_time and not rospy.is_shutdown():
		pub.publish(move_cmd)


	#After specific distance, stop robot 
	move_cmd.angular.z = 0 
	pub.publish(move_cmd)

	rospy.loginfo("Turned %.2f degrees.", distance * 180 / math.pi)

def get_yaw_from_quaternion(ori_q): 
	qu = (
		ori_q.x,
		ori_q.y,
		ori_q.z,
		ori_q.w
	)
	_, _, yaw = tf.transformations.euler_from_quaternion(qu)
	return yaw

def odom_get():
	try:
		rospy.sleep(0.2)
		odom_msg = rospy.wait_for_message('/odom', Odometry, timeout=5)
		position = odom_msg.pose.pose.position
		orientation = odom_msg.pose.pose.orientation
		yaw = get_yaw_from_quaternion(orientation)

		return (position.x, position.y, position.z, yaw)
	except rospy.ROSException:
		rospy.logwarn("Timeout while waiting for odom msg.")

def odom_delta_print(start, end):
	dx = end[0] - start[0] 
	dy = end[1] - start[1]
	dz = end[2] - start[2]
	dyaw = end[3] - start[3]

	dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
	rospy.loginfo('Odometry Change:')
	rospy.loginfo(' dx = %.3f m:', dx)
	rospy.loginfo(' dy = %.3f m:', dy)
	rospy.loginfo(' dz = %.3f m:', dz)
	rospy.loginfo(' dyaw = %.3f rad (%.1f deg):', dyaw, math.degrees(dyaw))


if __name__ == '__main__':
	try:
		start = odom_get()
		if start is None:
			rospy.loginfo("Failed to get initial position.")	
		move_fwd(float(sys.argv[1]), float(sys.argv[2]))
		end = odom_get()
		if end is None:
			rospy.loginfo("Failed to get final position.")	

		odom_delta_print(start,end)
	except rospy.ROSInterruptException:
		pass
