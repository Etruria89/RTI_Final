#!/usr/bin/env python

import rospy
import random
from sol_1_pkg.srv import Target, TargetResponse

"""
This script creates a service node for the generation of 
the new targets for the robot
 
...
    
Function
-----------
target_rand(req): fills the server placeholders x_target and y_target
    with a random number between -6.0 and 6.0

"""

def target_rand(req):

    """
    Parameters:
    ----------
    req : is called with Target request (empty)
          and return instances of Target response

    see Target srv:
    ---
    float32 x_target
    float32 y_target

    when called fills the server placeholders x_target and y_target
    with a random number between -6.0 and 6.0 

    """
     
    rospy.loginfo('NewTaget called')    
    x_target = random.uniform(-6.0, 6.0)
    y_target = random.uniform(-6.0, 6.0)
    print("Server answer")
    print(x_target, y_target)
    return TargetResponse(x_target,y_target)


rospy.init_node('new_target')
#Server istance
rospy.Service('new_target', Target, target_rand)
rospy.spin()


