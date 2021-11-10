# Experimental Robotics Laboratory - Assignment 1
This project implements a simplified version of the game Cluedo where a robot moves through different rooms looking for hints to find the murderer, the weapon and the location of the crime. 
## Expected Behaviour
The robot should:
- Explore the environment by entering in different rooms
- In each room, it should look around to find hints to make hypotheses
- When a consistent hypothesis is deducible, it should go in a designed location and express it in English
- If the hypothesis is wrong, it should keep exploring and find new hints
# Project structure
For this project I've implemented 4 nodes: the main node that simulates our robot behaviors [robot.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/robot.py), the node that will take care of the movement of the robot between the rooms [move_to_room.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/move_to_room.py), the oracle that knows all the hints and hypothesis and provides the hints to the robot during the game [oracle.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/oracle.py) and finally the [hint.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/hint.py) node that takes care of the communication with the ontology with the purpose of putting together the clues and deciding if they form a complete and consistent hypothesis to be sent to the robot.
## Nodes
- [robot.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/robot.py): inside to this node I've only two functions, the main function that implement the behaviors of the robot and the finction hypothesis_check that is called when the the subscriber sub take a new hypothesis from the topic /hypothesis. But let me spent some words for the main function, inside the function I initialise the publisher for the topic /reach and the subscriber for the topic /hypothesis. Inside the while loop I take from the paramters the value of the state that can be 0 or 1. If the value is 0 the robot starts to move it inside the environment, it choses random a room and using the service move_to_room (that for the moment only takes the values of the target and after a few seconds reply to the robot node with a boolean True) it goes in the new room. When it reach the room it publish on the topic /reach and the oracle send a random hint to the hint node. It do this beahviour until the hint node, with the founding hints, finds a complete and consitent hypothesis. When it heppens the state change in 1 and the robot do the second behavior, set the new target position to the oracle room, then it reach it and it tells to the oracle its sentence, with the service /Check_id the Oracle tells to the robot if the hypothesis is the right one. If is the right one the game finishes instead if is wrong the robot starts again with the first behavior
- [move_to_room.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/move_to_room.py): inside this node I've implemented a very simple and primitive go_to_point function, for the moment this node implement the server /Change_room that received a empty request from the robot, takes the value of the target from the parameters and after a certain time reply with a boolean variable set to True
- [oracle.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/oracle.py): inside this node I've implemented two functions, the id_winner function and the send_hints function. When the subscriber sub of the topic /reach received that the robot is in the room, it publishes on the topic /hint the string with the hint inside (e.g. ID1:where:Hall). The second function, id_winner, it's a server function that check the ID of the hypothesis set by the robot on the parameters. If the ID found fromt he robot is the same of the winning ID it sends to the robot a boolean sentece True otherwise it sends False
- [hint.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/hint.py)
## Messages
* `exp_ass1/Reach`: this message with topic **/reach** contain one boolean variable with name done, the publisher is the node [robot.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/robot.py) and the subscriber is the the node [oracle.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/oracle.py). I used this message to communicate to the oracle that the robot is inside the room and ready to receive a new hint
* `exp_ass1/Hint`: this message with topic **/hint** contain a string variable with name hint, the publisher is the node [oracle.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/oracle.py) and the subscriber is the node [hint.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/hint.py). I used this message to send with the oracle a new hint (as ID3:what:Revolver) to the hint node that checks and handles it
* `exp_ass1/Hyp`: this message with topic **/hypothesis** contain four strings that make up the hypothesis, ID, who, what and where. The publisher is the node [hint.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/hint.py) and the subscriber is the node [robot.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/robot.py). I used this node to send a complete and consistent hypothesis to the robot
## Services
* Inside of the [oracle.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/oracle.py) script there is the `/Check_id` service: it provides a boolean value. Request/reply interactions are done via a service file [Check_id.srv](https://github.com/piquet8/exp_ass1/blob/master/srv/Check_id.srv) which has an empty request field and  a boolean response field. When the robot calls this service the oracle node check if the ID found is the winning ID
* Inside of the [move_to_room.py](https://github.com/piquet8/exp_ass1/blob/master/scripts/move_to_room.py) script there is the `/Change_room` service: it provides a boolean value. Request/reply interactions are done via a service file [Move_to_room.srv]https://github.com/piquet8/exp_ass1/blob/master/srv/Move_to_room.srv which has an empty request field and a boolean response field. When the robot calls this service the move_to_room node takes the target position and sends a boolean value True.
## Parameters
They are inside of the [ass1_exprob.launch](https://github.com/piquet8/exp_ass1/blob/master/launch/ass1_exprob.launch) and are:
- `state` for the individuation of the current state of the robot, it can be 0 or 1
- `ID` is the value identifying of the hypothesis 
- `WHO` is the value of the person of the hypothesis
- `WHAT` is the value of the weapon of the hypothesis
- `WHERE` is the value of the place of the hypothesis
- `room_x` for memorize the coordiantes x of the target that the robot have to reach
- `room_y` for memorize the coordiantes y of the target that the robot have to reach
## Rqt-graph
Here we can see the graphs showing what's going on in the system, the first with Nodes only and the second with Nosdes and Topic:
![Rqt-graph](https://github.com/piquet8/exp_ass1/blob/master/rqt_graph/rqt_graph_1.png) 
![Rqt-graph](https://github.com/piquet8/exp_ass1/blob/master/rqt_graph/rqt_graph_2.png)
# How to launch
1. Firstly, open the terminal, go to your workspace and in the src folder run:
```
git clone https://github.com/piquet8/exp_ass1.git
```
**Remember** that python files must be made executable before they are run, to do this, go to the directory of the file and use: `chmod +x file_name.py`

2. Then to launch the program open a new shell tab and run the command:
```
roslaunch exp_ass1 ass1_exprob.launch
```
## Screenshot of the running programme
Here some screenshots showing the relevant parts of the running code:
- In these first two screenshots we simply see what happens when we launch our programme using the command roslaunch exp_ass1 exprob_ass1.launch. The parameters contained in the .launch file are shown, initially set with generic values chosen by me. Under the parameters the nodes are shown, all the nodes except the move_to_room and armor_service node have output screens so they show the terminal what they print. I chose to show all three because each of them plays an important role. Finally, we can see the rosmaster and individual nodes starting up in sequence

![launch](https://github.com/piquet8/exp_ass1/blob/master/screenshots/1_launch.png) 

![nodes_start](https://github.com/piquet8/exp_ass1/blob/master/screenshots/2_nodes%20start.png) 

- In this screen instead we can see the rules of the game written to inform the user of what is going to happen, this message will be displayed for a short period of time after which the robot will start to move

![cluedo_rules](https://github.com/piquet8/exp_ass1/blob/master/screenshots/3_cluedo%20rules.png)

- In this screenshot we see the robot in its first possible behaviour, that is the searching for hints. As can be seen, the robot randomly chooses coordinates from the possible ones that correspond to possible rooms. So the robot goes to a room and once there finds a hint. Each time the hint is checked and handled by the hint node. The printouts that we see are of course not done all at the same time but are interspersed with the various messages and services named above

![search](https://github.com/piquet8/exp_ass1/blob/master/screenshots/4_search_behavior.png)

- In this screenshot we see the robot in the second behaviour, it has received a possible valid hypothesis, so it goes to the oracle room and proposes its hypothesis.
Since the correct hypothesis is set as ID3, obviously the oracle with the Check_id service will return a boolean False and the answer Wrong to the robot. Then the robot will resume searching fors hints and thus to the behaviour seen in the previous screenshot

![wrong](https://github.com/piquet8/exp_ass1/blob/master/screenshots/5_wrong_hypothesis.png)

- In this last screenshot we see the same situation as before but this time the hypothesis found is the winning one, so the oracle will return the boolean True and tell the robot Right. 
In this case, the script exits the loop and the programme ends.

![winner](https://github.com/piquet8/exp_ass1/blob/master/screenshots/6_right_hypothesis.png)





