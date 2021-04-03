#!/usr/bin/env python

import rospy
import math
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sol_1_pkg.srv import Target

"""
This script creates a client for the control of the robot

....

Functions:
--------------

my_odom(msg): read the robot position from the Odometry nav_msgs.msg
        topic

main(): Control robot velocity and target achievemetn.

Initialization:
--------------

Initial position of the robot:
	x_pose = 0.0 (global variable)
	y_pose = 0.0 (global variable)
Threshold between target and robot to ensure the target achievement
	d_th = 0.1 (global parameter)
Gain on the speed value proportional to the distance between robot
and target
	speed_gain = 0.5 (global parameter)

"""



#Initialize the pose
x_pose = 0.0
y_pose = 0.0
#Initialize the threshold on the distance to stop the robot
d_th = 0.1
speed_gain = 0.5

#Here the callback to read the position
def my_odom(msg):

    """
    Parameters:
    ----------
    msg : it subscibes to the topic Odometry
          with an instance msg

    when subscribed provide access to robot positon
    updates the global variables x_pose and y_pose

    """
    #Define global varaibles
    global x_pose
    global y_pose
    #Read the values
    x_pose = msg.pose.pose.position.x
    y_pose = msg.pose.pose.position.y

def main():

    """Control robot velocity and target achievement.

    When the robot simulation is active it 
    subscribes to the odometry topic and
    changes the robot speed 
    publishing on the topic /cmd_vel. 
    It checks the relative
    position between the robot and the target.
    When the target is reached it makes a request to the 
    service Target to get a new target.

    Parameters:
    ----------
    none 
     
    """
    
    #Create a subscriber to the odometry
    odo_sub = rospy.Subscriber("odom", Odometry,my_odom)
    #Create a publisher to update the speed of the robot
    pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    #Create the service client  
    restart = rospy.ServiceProxy('new_target', Target)

    while not rospy.is_shutdown():	
        
     	# Careate an istance of the client
    	print("New target requested.")
    
    	rospy.wait_for_service('new_target')
    	try: 
    	    rst = restart()
	    print("New target:")
    	    x_target = rst.x_target 
    	    y_target = rst.y_target
    	    print(x_target, y_target)
    	except rospy.ServiceException as e:
	    print("Service call failed: %s" %e)   
	
    	#Estimate the distance in between the robot and the target	
    	dist_x = (x_target-x_pose)
    	dist_y = (y_target-y_pose)    
    	dist = math.sqrt(dist_x**2 + dist_y**2)
    
     
    	# while the distance is below the threshold
    	# evaluate the distance in between the target and the robot and update the speed
    	while dist > d_th:

            # evaluate the speed
            speed_x = speed_gain * dist_x
            speed_y = speed_gain * dist_y

      	    #publish the speed
	    twist = Twist()
	    twist.linear.x = speed_x
	    twist.linear.y = speed_y        

	    pub.publish(twist)

            #Update robot positon	
    	    dist_x = (x_target-x_pose)
    	    dist_y = (y_target-y_pose)    
    	    dist = math.sqrt(dist_x**2 + dist_y**2)

    	#If the robot is close enough to the target 
    	#write that the target is reached and restart the loop asking the server for a new target
    	print('Target reached :)')
    
        
if __name__ == '__main__':

    try:
	#initialize the node
	# anonymous is True to ahve more than a listener
    	rospy.init_node("robot_controller", anonymous="True",disable_signals=True)
     	main()	
	rate.sleep()
    except KeyboardInterrupt:
	print("Out")
