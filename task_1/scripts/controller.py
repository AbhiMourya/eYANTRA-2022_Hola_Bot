#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math
from math import pi

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

hola_x = 0
hola_y = 0
hola_theta = 0
x_goals =[]
y_goals =[]
theta_goals =[]

def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)
def odometryCb(msg):
    global hola_x, hola_y, hola_theta

    # Write your code to take the msg and update the three variables 
    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y
    hola_theta = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]

def main():
	# Initialze Node
    rospy.init_node('controller',anonymous=True)
    rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
    rospy.Subscriber('/odom',Odometry,odometryCb)
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively

	# Declare a Twist message
    # Declare a Twist message
    vel = Twist()
    vel_x = 0
    vel_y = 0
    error_thetab = 1
	# Initialise the required variables to 0
	# <This is explained below>
	
	# For maintaining control loop rate.
    rate = rospy.Rate(100)

	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
    # x_d,y_d,theta_d=-3,-7,math.pi/3
    # x_goals=[1,-1]
    # y_goals=[0,1]
    # theta_goals=[0,0]
    global x_goals,y_goals,theta_goals
    
	# and also Kp values for the P Controller
    
    while not rospy.is_shutdown():
        
        for (x_d,y_d,theta_d) in zip(x_goals,y_goals,theta_goals):
            # Find error (in x, y and theta) in global frame
            global hola_x, hola_y, hola_theta
            
            move(x_d,y_d,vel,pub,0.08,1)
            turn(theta_d,vel,sgm,pub,0.1,1.5)
            move(x_d,y_d,vel,pub,0.01,0.1)
            turn(theta_d,vel,sgm,pub,0.01,0.5)
            move(x_d,y_d,vel,pub,0.005,0.05)
            turn(theta_d,vel,sgm,pub,0.0001,0.05)
            rospy.sleep(2)
            rate.sleep()

def sgm(value):
        if value>0:
            return 1
        elif value<0:
            return -1
        else:
            return 0
def turn(angle,msg,sgm,pub,er,speed):
    global hola_theta
    err=angle-hola_theta
    
    while(abs(err)>er):
        err=angle-hola_theta
        msg.angular.z = speed*sgm(err)
        pub.publish(msg)
    msg.angular.z = 0
    pub.publish(msg)

def move(x,y,vel,pub,er,speed):
    global hola_x,hola_y
    vel_x = x - hola_x
    vel_y = y - hola_y
    while(abs(vel_x)>er or abs(vel_y)>er):
        
        vel_x_new = vel_x*math.cos(hola_theta) + vel_y*math.sin(hola_theta)
        vel_y_new = -vel_x*math.sin(hola_theta) + vel_y*math.cos(hola_theta)
        r=(vel_x_new**2+vel_y_new**2)**.5
        vel.linear.x = speed*vel_x_new/r
        vel.linear.y = speed*vel_y_new/r
        pub.publish(vel)
        vel_x = x - hola_x
        vel_y = y - hola_y
        
    vel.linear.x = 0
    vel.linear.y = 0
    pub.publish(vel)

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

    # try:
    #     main()
	# except rospy.ROSInterruptException:
	# 	pass


		# (Calculate error in body frame)
		# But for Controller outputs robot velocity in robot_body frame, 
		# i.e. velocity are define is in x, y of the robot frame, 
		# Notice: the direction of z axis says the same in global and body frame
		# therefore the errors will have have to be calculated in body frame.
		
		# This is probably the crux of Task 1, figure this out and rest should be fine.

		# Finally implement a P controller 
		# to react to the error with velocities in x, y and theta.

		# Safety Check
		# make sure the velocities are within a range.
		# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
		# we may get away with skipping this step. But it will be very necessary in the long run.