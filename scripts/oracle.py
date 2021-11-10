#!/usr/bin/env python

## @package exp_ass1
#
#  \file oracle.py
#  \brief This program simulate the movement of the robot from a room to another one
#
#  \author Gianluca Piquet
#  \version 1.0
#  \date 04/11/2021
#  \details
#  
#  Subscribes to: <BR>
#       /reach
#
#  Publishes to: <BR>
#	/hint	    
#
#  Services: <BR>
#       /Check_id
#
#  Client Services: <BR>
#       None
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#    This node is the Oracle, it implements two important function, the function sed_hints that 
#    sends random hint to the node hint that upload and check them in the ontology, the function
#    id_winner instead check if the hypothesis that the robot found is the correct one.

import rospy
import random
from std_srvs.srv import *
from exp_ass1.srv import *
from exp_ass1.msg import *



# room = 'Conservatory', 'Lounge', 'Kitchen', 'Library', 'Hall'
# weapon = 'Candlestic', 'Dagger', 'Revolver', 'Rope', 'Spanner'
# person = 'Prof.Plum', 'Rev.Green', 'Col.Mustard', 'Mrs,Peacock', 'Miss.Scarlett'

# inside the array there are all the possible hints that the robot can find
hints = ['ID1:where:Conservatory', 'ID1:what:Dagger','ID1:where:Kitchen','ID2:who:Prof.Plum', 'ID2:what:Spanner','ID2:where:Hall','ID3:where:Lounge', 'ID3:what:Revolver', 'ID3:who:Miss.Scarlett','ID4:where:Library','ID4:what:Candlestic','ID4:who:Rev.Green','ID5:what:Rope','ID5:who:Col.Mustard', 'ID5:who:Mrs,Peacock']

# initialise the winner ID
id_win = 'ID3'

##
#	\brief This function pubblishes the hint chosed randomly on the topic /hint
#	\param : msg, the data that the orcle publish on the topic /hint for the hint node 
#	\return : None 
#

def send_hints(msg):

	pub=rospy.Publisher('/hint', Hint, queue_size=1000)
	n_hint = random.randint(0,14)
	msg = Hint()
	msg.hint = hints[n_hint]
	pub.publish(msg)

##
#	\brief This function check if the ID found by the robot is the winner and sends the 
# result as reply
#	\param : req, request from the node robot that call this server
#	\return : resp, contains a boolean value True if the ID is the winner
#
# This function take the value of the ID hypothesis found by the robot and check if the valure
# is the same of the id_win, the id of our winning hypothesis. If the value is the same it sends
# to the robot a reply with a boolean reault True, if is wrong it sends the result False

def id_winner(req):
	
	resp = Check_id() 
	id_try = rospy.get_param("ID")
	if id_try == id_win:
		resp = True
	else:
		resp = False
	return resp

##
#	\brief This function implements the ros node
#	\param : None
#	\return : None 
#
# This function initialises the node, the subscriber for the topic /reach and the service /
# Check_id

def main():
	
	#initialising the node
	rospy.init_node('oracle')
	
	# initialising the subscriber on the topic /reach
	sub=rospy.Subscriber('/reach', Reach, send_hints)
	
	# initialising the service /check_id
	srv = rospy.Service('/Check_id', Check_id, id_winner)
	
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
		rate.sleep()

if __name__=='__main__':
	main()
