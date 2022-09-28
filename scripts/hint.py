#! usr/bin/env python2

## @package exp_ass1
#
#  \file hint.py
#  \brief This program simulate the ontology of the game
#
#  \author Gianluca Piquet
#  \version 1.0
#  \date 04/11/2021
#  \details
#  
#  Subscribes to: <BR>
#       /hint
#
#  Publishes to: <BR>
#	/hypotheis	    
#
#  Services: <BR>
#       None
#
#  Client Services: <BR>
#	armor_interface_srv
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#    This node receives the hints published on the topic hint from the oracle node. Then it    
#    check them: if the hint is not already present in the ontology it adds it.                 
#    After that it checks if the hint has already been saved in the ontology, if it is the 
#    first time the hint has been received it checks if the hypothesy that correspond to that  
#    hint ID is complete and not inconsistent. In case the hypothesis is right it sends it to
#    the robot. When one hint belongs to a hypothesis already check the node print Manage 
#    Hypothesis: information source with this ID already checked   

import copy
import math
import time
import rospy

import geometry_msgs.msg
from exp_ass1.msg import *

from armor_msgs.msg import * 
from armor_msgs.srv import * 

# initialising the global variables

armor_service = None
pub= None

people=[]
weapons=[]
locations=[]
hypothesis=[]
 
##
#	\brief This function loads the ontology from a file called cluedo_ontology in a 
# specific 
#	\param : None
#	\return : None 
#
 

def load_file():
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'LOAD'
        req.primary_command_spec= 'FILE'
        req.secondary_command_spec= ''
        req.args= ['/root/ros_ws/src/exp_ass1/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)

##
#	\brief This function adds one entity to the ontology.  
#	\param : name is the instance name
#	\param : class_type represent the type of the instance among who, what and where
#	\return : None 
# 

def add_instance(name, class_type):
    try:
        class_id=find_type(class_type)
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [name, class_id]
        msg = armor_service(req)
        res=msg.armor_response
        reason()
        disjoint(class_id)
        reason()
    except rospy.ServiceException as e:
        print(e)

##
#	\brief This function checks the type of the class among who, what and where and returns # the string 
# with the corresponding class name.
#	\param : class_type
#	\return : class name
#	


def find_type(class_type):
    if class_type=='who':
        return 'PERSON'
    if class_type== 'what':
        return 'WEAPON'
    if class_type=='where':
        return 'LOCATION' 

##
#	\brief This function calls the armor server in order to run the reasoner 
#	\param : None
#	\return : None 
#    

def reason():
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REASON'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= []
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)		
 
##
#	\brief This function calls the armor server and by sending specific commands it 
# specifies that all 
# entities inside the class passed as parameter are disjoint and different  
#	\param : class type of element that I want disjoint
#	\return : None 
#	
 

def disjoint(class_type):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [class_type]
        msg = armor_service(req)		 
    except rospy.ServiceException as e:
        print(e)        

##
#	\brief This function, for each element of the list passed as input it splits the strings
#	\param : query , list of strings that we want clean
#	\return : query, query after clean 
#	
# This function, for each element of the list passed as input it splits the strings at the char 
# '#' and takes only what is after. Then it removes the last element of the remaing string.

def clean_queries(query):
    for i in range(len(query)):
        temp=query[i]
        temp=temp.split('#')
        index=len(temp)
        temp=temp[index-1]
        query[i]=temp[:-1]
    return query

##
#	\brief In this function, depending on the type of string that has been received, it 
# checks in the 
# corresponding global list if already present
#	\param : data, list that want to check if already received
#	\return : find 
#
# In this function, depending on the type of string that has been received, it checks in the 
# corresponding global list if already present. In case it then it at the end.   

def check_if_received_before(data):
    global people, weapons, locations
    find=0
    i=0
    j=0
    k=0
    if data[1]=='who':
        for i in range(len(people)):
            if people[i]==data[2]:
                find=1;
        if find==0:
            people.append(data[2])
    if data[1]=='what':
        for j in range(len(weapons)):
            if weapons[j]==data[2]:
                find=1;
        if find==0:
            weapons.append(data[2])
    if data[1]=='where':
        for k in range(len(locations)):
            if locations[k]==data[2]:
                find=1;
        if find==0:
            locations.append(data[2])
    return find            

##
#	\brief This function adds an entity to a given hypothesis. 
#	\param : ID, id of the hypothesis that we want to add
#	\param : class_type, type of information that we want add
#	\param : name, name of information that we want add
#	\return : None 
#


def add_hypothesis(ID,class_type,name):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID,name]
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)	

##
#	\brief  This funciton calls the armor server to see in a given hypothesis identified by # its ID
#	\param : ID, id of the hypothesis that we want to check
#	\param : class_type, type of information that we want to retreieve
#	\return : res_final, name of entity retrieved 
#	


def look_hypothesis(ID,class_type):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID]
        msg = armor_service(req)
        res=msg.armor_response.queried_objects
        res_final=clean_queries(res)
        return res_final
    except rospy.ServiceException as e:
        print(e)   

##
#	\brief This function checks if a hint is already present in an hypothesis.
#	\param : ID, id of the hypothesis that we want to check
#	\param : class_type, type of information that we want to check
#	\return : None 
#
# This function checks if a hint is already present in an hypothesis. If case it adds it to the # ontology.

def check_in_ontology(ID,class_type,name):
    try:
        namef=[]
        res_final=look_hypothesis(ID, class_type)
        namef.append(name)
        if res_final != namef:
            add_hypothesis(ID,class_type,name)
            reason()
    except rospy.ServiceException as e:
        print(e)      

##
#	\brief This function calls the armor server twice
#	\param : ID, hypothesis identifier
#	\return : 1 if hypothesi complete and consistent 0 if not
#
# This function calls the armor server twice, the first time it retrieves all the complete 
# hypothesis and it checks if the searched hypothesis is in that list, it then does the same to 
# check if the hypothesis is inconsistent.

def check_complete_consistent(ID):
    #check completed
    try:
        completed=0
        inconsistent=0
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['COMPLETED']
        msg = armor_service(req)
        res=msg.armor_response.queried_objects
        res_final=clean_queries(res)
        for i in range(len(res_final)):
            if res_final[i]==ID:
                completed=1
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['INCONSISTENT']
        msg = armor_service(req)
        res=msg.armor_response.queried_objects
        res_final=clean_queries(res)
        for i in range(len(res_final)):
            if res_final[i]==ID:
                inconsistent=1
        if completed==1 and inconsistent==0:
            return 1
        else :
            return 0
    except rospy.ServiceException as e:
        print(e)

##
#	\brief This function is called when new hint arrived
#	\param : msg, from topic /hint
#	\return : None 
#
# This function elaborates all of the hints received and decides weather or not to make an 
# hypothesy. If the hypothesy is complete and consistent, and if it was never published before 
# it is sent on the topic /hypothesis as a message of type Hypothesis.msg, that has 4 fields 
# each of string type: ID, who, what, where to the robot node

def send_hint(msg):
		
    global hypothesis
    
    already_checked=0
    tip=(msg.hint)
    
    print ('ROBOT: I found a new hint ' + tip)
    hint_received=tip.split(':')
    rospy.set_param('ID', hint_received[0])
    find=check_if_received_before(hint_received)
    if find==0:
        add_instance(hint_received[2],hint_received[1])	    
    check_in_ontology(hint_received[0], hint_received[1], hint_received[2])
    send=check_complete_consistent(hint_received[0])
    
    if send==1:
        for i in range(len(hypothesis)):
            if hint_received[0]==hypothesis[i]:
                already_checked=1
                print('Manage Hypothesis: information source with this ID already checked \n')
        if already_checked==0:
            print('Manage Hypothesis: complete and consistent. It coould be the right hypothesis..\n')
            hypothesis.append(hint_received[0])
            message= Hyp()
            message.ID=hint_received[0]
            temp=look_hypothesis(hint_received[0], 'who')
            message.who=temp[0]
            temp=look_hypothesis(hint_received[0], 'what')
            message.what=temp[0]
            temp=look_hypothesis(hint_received[0], 'where')
            message.where=temp[0]
            print(message)
            pub.publish(message)
    else:
        print ('Manage Hypothesis: not complete or not consistent')
 
##
#	\brief This function implements the ros node
#	\param : None
#	\return: None
#
       
def main():
  global  armor_service, pub
  
  # initialise the node
  rospy.init_node('Hint')
  
  # call the armor service
  armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
  
  # publisher on the topic /hypithesis
  pub = rospy.Publisher('/hypothesis', Hyp, queue_size=1000)
  
  rospy.wait_for_service('armor_interface_srv')
  
  #subscirber for the topic /hint
  sub = rospy.Subscriber('/hint', Hint, send_hint)

  
  load_file()
  rospy.spin()

if __name__ == '__main__':
  main()        
