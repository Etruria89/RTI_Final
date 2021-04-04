#! /usr/bin/env python

# import ros stuff
import rospy
import random
import time
from std_srvs.srv import *
from final_assignment.srv import Target

"""
This script handles the user iterface node for selecting the robot behaviour
 
...
    
Functions
-----------
user_interface(): user interface function
main() : main code for the server definition

"""

# service callback
target_array = [[-4,-3],[-4,2],[-4,7],[5,-7],[5,-3],[5,1]]

#Iitialize the client for requesting the target_id 
restart = rospy.ServiceProxy('rand_target_id', Target)
rospy.init_node('user_interface')

def user_interface(req):


    """ The function prints on the command line a user interface 
	for the definition of the next action for the robot

	The function request the user an integer to specify the new desired behaviour
        (beh_int) and additional parameters to specify the required action.
	The relevant parameters in the ros parameter server.	 

    ------

    Parameters
    -----------
    des_pos_x : float that defines the old x postion of the target read from the parameter list 
    des_pos_y : float that defines old y postion of the target read from the parameter list 
    state_value : integer set to required behaviour in the parameter server 
    target_time : positive float value that specify the required amout of time for the wall_following [3] and the keep position [2] action
	          it is set via teh user interface
    bug_trigger : Boolean that specifies if the bug_0 algorithm is active (1) or inactive (0)
    

    Internal variables
    -------------------
    beh_int: integer to specify the desired robot behaviour:
	[1] Random target position requested to the "/target_provider" server 
	[2] User defined new target via the integer input "tar_id"
	[3] Wall follow behaviour for "wall_time" seconds
	[4] Keep the position for "wall_time" seconds
	[5] Move_base to Bug_0 path planning algorithm switch
    """ 

    time.sleep(1)
    print("Please select the new behaviour:")
    print("[1] Random target chosen in between [(-4,-3),(-4,2),(-4,7),(5,-7),(5,-3),(5,1)]")
    print("[2] Select a target in between [(-4,-3),(-4,2),(-4,7),(5,-7),(5,-3),(5,1)]")
    print("[3] Start following the external walls")
    print("[4] Keep the position")
    print("[5] Change the planning algorithm")
    beh_int = int(raw_input('I :'))
   
    if beh_int < 3:
	
	x_old = rospy.get_param('des_pos_x')
	y_old = rospy.get_param('des_pos_y')
	x = x_old
        y = y_old
	min_id = 1
 	max_id = 6

	while x_old == x and y_old ==y:	
            if beh_int == 1:
		target = restart()
    	    	target_id = target.target_id
		print('Service called:')
		print(target_id) 
	        target = target_array[target_id-1]
            elif beh_int == 2:
	    	print("Choose your target [(-4,-3),(-4,2),(-4,7),(5,-7),(5,-3),(5,1)]")
	    	print("Choose a value of the target id in between 1 and 6")
            	tar_id = int(raw_input('Id :'))
	    	if tar_id < min_id or tar_id > max_id:
          	    print("Invalid entry selected") 
		    target = [x_old, y_old]		    	                   
		else:	
	    	    target = target_array[tar_id-1]
    	    x = float(target[0])
    	    y = float(target[1])
 	    if x_old == x and y_old == y:
	    	print("I am here. Please, give me a new location.")    
	
	
    	rospy.set_param("des_pos_x", x)
    	rospy.set_param("des_pos_y", y)
	state = rospy.set_param('state_value', 0)
    	print("Thanks! Let's reach the next position")	
	print(x,y)

    elif beh_int == 3: 
	       
	print("How many seconds do you want me to follow the wall?")
	print("Please, enter a positive value.")
        wall_time = int(raw_input('w_t:'))
	print("I will follow the walls for %d seconds." % wall_time)	
	rospy.set_param('target_time', wall_time)

	state = rospy.set_param('state_value', 3)
	
    elif beh_int == 4:
        print("How many seconds do you want me to keep this position?")
	print("Please, enter a positive value.")
        wait_time = int(raw_input('w_t:'))
	print("I will stay here for %d seconds." % wait_time)
	rospy.set_param('target_time', wait_time)
	state = rospy.set_param('state_value', 4)

    elif beh_int == 5:

	# Read the current algorithm active
	bug_status = rospy.get_param('bug_trigger')
	print(int(bug_status))
	if bug_status == 0:
	    print('Move_base disabled')
	    rospy.set_param('bug_trigger', 1)
	    print('Bug_0 enabled')
	else:
	    print('Bug_0 disabled')
	    rospy.set_param('bug_trigger', 0)
	    print('Move_base enabled')
	print(rospy.get_param('bug_trigger'))
	state = rospy.set_param('state_value', 5)

    return []

def main():
    
    """ The node handles for the user interface.

	It handles the set of the user interface 

    ------

    Parameters
    -----------
    des_pos_x: x postion of the target read from the parameter list need for the first target definition
    des_pos_y: y postion of the target read from the parameter list need for the first target definition

    """ 
    x = rospy.get_param("des_pos_x")
    y = rospy.get_param("des_pos_y")
    print("Hi! We are reaching the first position: x = " +
        str(x) + ", y = " + str(y))
    rospy.Service('user_interface', Empty, user_interface)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown(): 	
        rate.sleep()


if __name__ == '__main__':
    main()
