#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_0.py
# Functions:
# 					[ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node


####################### IMPORT MODULES #######################

import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
##############################################################

def callback(data):
	
	# Input Arguments:
    global ANGLE,Y
    ANGLE  = data.theta*180/3.1415 
    Y= data.y
	# Returns:
    # rospy.loginfo("Current ANGLE:%f",ANGLE)
    # rospy.loginfo("Current position y:%f",y)


def main():
	
    rospy.init_node('turtle_controller_node', anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, callback)
   
    rospy.loginfo("Start")

    controller = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
    obj_msg = Twist()


    while not rospy.is_shutdown():
          
        circle_move(start_move,obj_msg,controller,180)
        
        turn(start_move,obj_msg,controller,90)        

        straight_move(start_move,obj_msg,controller,2)

        rospy.loginfo(">>>Task completed<<<")
        print("Exiting......")
     
        rospy.signal_shutdown("completed")

        

    # rospy.spin()
        
            
                   

    
	


################# ADD GLOBAL VARIABLES HERE #################
ANGLE = 0
Y = 0

##############################################################


################# ADD UTILITY FUNCTIONS HERE #################
def start_move(pub,msg,speed_linear,speed_angular):
    msg.linear.x=speed_linear
    msg.angular.z=speed_angular
    pub.publish(msg)


def turn(action,msg,pub,turn_ANGLE):
    global ANGLE
    a0=ANGLE
    while abs(abs(a0)-abs(ANGLE))<=turn_ANGLE:
        rospy.loginfo("Turning...")
        action(pub,msg,0,1)
    action(pub,msg,0,0)
    # global Y
    # print(Y)

def circle_move(action,msg,pub,ANGLE_covered):
    global ANGLE
    a0=ANGLE
    while abs(a0-ANGLE)<=ANGLE_covered-0.5:
        rospy.loginfo("Moving in circle...")
        action(pub,msg,1,1)
    action(pub,msg,0,0)

def straight_move(action,msg,pub,destination_y):
    global Y
    y0=Y
    while abs(y0-Y)<=destination_y:
        rospy.loginfo("Moving in straight line... ")
        action(pub,msg,1,0)
    action(pub,msg,0,0)




##############################################################


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()

    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")
