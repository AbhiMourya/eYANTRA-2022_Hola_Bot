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
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"
	
	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively
    # declare that the node subscribes to task1_goals along with the other declarations of publishing and subscribing
    rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
    rospy.Subscriber('/odom',Odometry,odometryCb)
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

	# Declare a Twist message
    vel = Twist()
    vel_x = 0
    vel_y = 0
    vel_z = 0
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
        # print("i am here")
        for (x_d,y_d,theta_d) in zip(x_goals,y_goals,theta_goals):
            # Find error (in x, y and theta) in global frame
            global hola_x, hola_y, hola_theta
            e_x=1
            e_y=1
            while(abs(theta_d-hola_theta)>=0.05):
                    err=theta_d-hola_theta
                    vel.angular.z=(err)/abs(err)
                    pub.publish(vel)
            vel.angular.z=0
            pub.publish(vel)


            while (abs(e_y)>=0.01 or abs(e_x)>=0.04):
                # print(e_x,",",e_y)
                e_x= x_d - hola_x +1e-4
                e_y= y_d - hola_y +1e-4
                e_theta= theta_d - hola_theta
                            
                r=(e_x**2+e_y**2)**0.5
                vel_x = 0.5*(e_x)/(r)
                vel_y = 0.5*(e_y)/(r)

                vel_x_new = vel_x*math.cos(hola_theta) + vel_y*math.sin(hola_theta)
                vel_y_new = -vel_x*math.sin(hola_theta) + vel_y*math.cos(hola_theta)

                vel.linear.x = vel_x_new
                vel.linear.y = vel_y_new
                pub.publish(vel)
            vel.linear.x =0
            vel.linear.y = 0
            pub.publish(vel)


            # the /odom topic is giving pose of the robot in global frame
            # the desired pose is declared above and defined by you in global frame
            # therefore calculate error in global frame

            # (Calculate error in body frame)
            # But for Controller outputs robot velocity in robot_body frame, 
            # i.e. velocity are define is in x, y of the robot frame,
            # Notice: the direction of z axis says the same in global and body frame
            # therefore the errors will have have to be calculated in body frame.
            # 
            # This is probably the crux of Task 1, figure this out and rest should be fine.

            # Finally implement a P controller 
            # to react to the error with velocities in x, y and theta.
            # re_theta=math.atan((y_d-hola_y)/(x_d-hola_x+10e-5))
            # re_theta=re_theta-hola_theta
            
            rospy.sleep(2)

            # Safety Check
            # make sure the velocities are within a range.
            # for now since we are in a simulator and we are not dealing with actual physical limits on the system 
            # we may get away with skipping this step. But it will be very necessary in the long run.
    rospy.signal_shutdown()   
    
        
        
        
    



if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
