#!/usr/bin/env python

import rospy
import random
from sol_1_pkg.srv import Target, TargetResponse


def target_rand(req):
     
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


