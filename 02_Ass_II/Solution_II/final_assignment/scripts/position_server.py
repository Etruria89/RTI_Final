#!/usr/bin/env python

import rospy
import random
from final_assignment.srv import Target, TargetResponse

"""
This script creates a service node for the generation of 
the new targets id for the robot
 
...
    
Function
-----------
target_rand(req): fills the server placeholder target_id with 
   a random number between 1 and 6

"""

def target_rand(req):

    """
    Parameters:
    ----------
    req : is called with Target request (empty)
          and return instances of Target response

    see Target srv:
    ---
    int32 target_id

    when called fills the server placeholders target id
    with a random integer number between 1 and 6 

    """
     
    rospy.loginfo('New Target id called:')    
    target_id = random.randint(1, 6)
    print("Server answers:")
    print(target_id)
    return TargetResponse(target_id)


rospy.init_node('rand_target_id')
#Server istance
rospy.Service('rand_target_id', Target, target_rand)
rospy.spin()


