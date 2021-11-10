#!/usr/bin/env python

## @package exp_ass1
#
#  \file move_to_room.py
#  \brief This program simulate the movement of the robot from a room to another one
#
#  \author Gianluca Piquet
#  \version 1.0
#  \date 04/11/2021
#  \details
#  
#  Subscribes to: <BR>
#       None
#
#  Publishes to: <BR>
#	None	    
#
#  Services: <BR>
#       /Move_to_room
#
#  Client Services: <BR>
#       None
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#   This node is only a simulation of the future node for the movement of the robot among the 
#   rooms. It take the values of the target position from the parameters set from the robot and 
#   then I will implement the go_to_point beahviour. For the moment when it is called it take the
#   values of the target, sleep for 5 and then send the reply that tell to the robot that it is 
#   arrived in the room

import rospy
import time
import math
# import ros service
from std_srvs.srv import *
from exp_ass1.srv import *
from exp_ass1.msg import *

##
#	\brief This function is called when robot receive a new target position /Move_to_room
#	\param : req, empty request
#	\return :  ok, boolean variable
#
#  This function simulate the movement of the robot, it takes the value of the target and then
#  sleep for 5, in this part I'll put the behavior go_to_point. Then when the robot will be
#  inside the room the function will send a boolean reply True

def change_room(req):
	
	ok = Move_to_room()
	
	x_position = rospy.get_param('room_x')
	y_position = rospy.get_param('room_y')
	
	print ('Move to x:' + str(x_position) + ' y:' + str(y_position))
	
	time.sleep(5)
	
	ok = True
	return ok
	
##
#	\brief This function implements the ros node
#	\param : None
#	\return : None 
#
# This function intialises the service /Change_room where the function change_room will be
# implemented as a go_to_point method to moves the robot in the environment 

def main():
	#initialise the node
	rospy.init_node('robot')
	
	#initialise the service
	srv = rospy.Service('/Change_room', Move_to_room, change_room)
	
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
		rate.sleep()
	
if __name__=='__main__':
	main()
	
	
	
