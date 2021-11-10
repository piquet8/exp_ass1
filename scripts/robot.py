#!/usr/bin/env python

## @package exp_ass1
#
#  \file robot.py
#  \brief This program simulate the behavior of the robot during the game
#
#  \author Gianluca Piquet
#  \version 1.0
#  \date 04/11/2021
#  \details
#  
#  Subscribes to: <BR>
#       /hypothesis
#
#  Publishes to: <BR>
#	/reach	    
#
#  Services: <BR>
#       None
#
#  Client Services: <BR>
#       /Change_room
#  Client Services: <BR>
#	/Check_id
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#  This node is the main, it implements the two behavior of the robot, in the first behavior
#  the robot goes around the room to find new hints, in the second behavior it goes to the
#  oracle with its hypothesis and check if it is the right one. If it's right it tells to us the
#  final sentence instead if is wrong it start again with the first behavior.

import rospy
import time
import random
import math
# import ros service
from std_srvs.srv import *
from exp_ass1.srv import *
from exp_ass1.msg import *


# oracle room:
oracle_position_x = 7
oracle_position_y = 7

# initialise the state to zero 
# state = 0 -> robot searches hints between the rooms
# state = 1 -> robot goes to the Oracle to check its hypothesis

state = 0

# possible target of the rooms: (2,2)Hall (2,4)Kitchen (7,2)Lounge (4,2)Library (4,4)Study (7,4)Conservatory 
x_targ = [ 2 , 4 , 7]
y_targ = [ 2 , 4]

##
#	\brief This function implements the ros node
#	\param : None
#
# This function implements the two behaviors of the robot, in the first part it initialises the 
# publisher for the topic /reach and the subscriber for the topic /hypothesis. Then it asks for
# the current state to the parameters and if state is 1 starts with the first behaviour. Set 
# the position for the room then asks to the service /Change_room if the robot is in the nwe 
# room when it happen it publish on the topic /reach. When the robot have a complete and 
# consistent hypothesis, it goes to the oracle again set the position in the same way as before
# and send a request to the service /Check_id to ask if the hypothesis is the right one.

def main():
  global position, srv_client_change_room, r_pub, srv_client_id_winner, state, inside, x , y
  
  # intitialise the node
  rospy.init_node('robot')
  
  # publisher on the topic /reach when the robot reaches a room
  r_pub = rospy.Publisher('/reach', Reach, queue_size = 1000)
  reach = Reach()
  
  # subsriber for the topic /hypothesis the hypothesis published from the hint node
  sub = rospy.Subscriber('/hypothesis', Hyp , hypothesis_check)
  hypothesis = Hyp()
  
  print('\n CLUEDO SIMULATION: \n - The robot goes around the rooms looking for a hint \n - When it finds a hypothesis consistent and complete using the hints it goes to the oracle to ask if its hypothesis is the right one \n - If its Right the game finish instead if its Wrong the robot starts again ti search \n')
  time.sleep(10)
  
  
  while not rospy.is_shutdown():
  	
  	# take the value of the state from the parameters
  	state = rospy.get_param('state')
  	
  	if (state == 0):
  		
  		# position of the room chosen randomly 
  		position_x = random.choice(x_targ)
  		position_y = random.choice(y_targ)
  		
  		# set the value of the target inside the parameters
  		rospy.set_param("room_x", position_x)
  		rospy.set_param("room_y", position_y)
  		
  		print('')
  		print ('ROBOT: Im going in the room in: x: ' + str(position_x) + ' y: ' + str(position_y))
  		
  		
  		if position_x == 2 and position_y == 2 :
  			room = 'Hall'
  		elif  position_x == 2 and position_y == 4 :
  			room = 'Kitchen'
  		elif  position_x == 7 and position_y == 2 :
  			room = 'Lounge'
  		elif  position_x == 4 and position_y == 2 :
  			room = 'Library'
  		elif  position_x == 4 and position_y == 4 :
  			room = 'Study'
  		elif  position_x == 7 and position_y == 4 :
  			room = 'Conservatory'
  		
  		# call the service change room
  		srv_client_change_room = rospy.ServiceProxy('/Change_room', Move_to_room)
  		new_room = srv_client_change_room()
  		
  		# reply from the server move_to_room when the robot reaches the room 
  		inside = new_room.ok
  		
  		# publish in the topic /reach
  		reach.done = True
  		r_pub.publish(reach)
  		
  		print ('ROBOT: Im in the ' + room)
  		time.sleep(2)
  	
  	elif (state == 1):
  		
  		print ('ROBOT: I have a hypothesis, im going to the oracle...')
  		
  		# set the value of the oracle's room inside the parameters
  		rospy.set_param("room_x", 7)
  		rospy.set_param("room_y", 7)
  		
  		srv_client_change_room = rospy.ServiceProxy('/Change_room', Move_to_room)
  		new_room = srv_client_change_room()
  		inside = new_room.ok
  		
  		print ('ROBOT: I reached the oracle\n')
  		
  		# take the values of the hypothesis from the parameters
  		ID_r = rospy.get_param("ID")
  		who_r = rospy.get_param("WHO")
  		what_r = rospy.get_param("WHAT")
  		where_r = rospy.get_param("WHERE")
  		
  		#call the service Check_id 
  		srv_client_id_winner = rospy.ServiceProxy('/Check_id', Check_id)
  		res = srv_client_id_winner()
  		
  		# True if the ID is the winner else False
  		result = res.resp
  		
  		time.sleep(3)
  			
  		if result == False:
  			print('It is ' + who_r + ' with the ' + what_r + ' in the ' + where_r)
  			time.sleep(1)
  			print('ORACLE: WRONG! \n')
  			rospy.set_param("state", 0)
  		else:
  			print('It is ' + who_r + ' with the ' + what_r + ' in the ' + where_r)
  			time.sleep(1)
  			print('ORACLE: RIGHT! \n')
  			print('')
  			break
  			
##
#	\brief This function set the values of the hint inside the parameters
#	\param : hypothesis
#	\return : set the four values of the hypothesis 
		
  	
def hypothesis_check(hypothesis):
	print('Checking the hypothesis... \n')
	
	#set the parameters of the hypothesis
	rospy.set_param("ID", hypothesis.ID)
	rospy.set_param("WHO", hypothesis.who)
	rospy.set_param("WHAT", hypothesis.what)
	rospy.set_param("WHERE", hypothesis.where)
	rospy.set_param("state", 1)
	

if __name__  == '__main__':
    main()
