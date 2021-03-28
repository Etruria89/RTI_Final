#!/usr/bin/env python

import rospy
import math
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sol_1_pkg.srv import Target


#Initialize the pose
x_pose = 0.0
y_pose = 0.0
#Initialize the threshold on the distance to stop the robot
d_th = 0.1
speed_gain = 0.5

#Here the callback to read the position
def my_odom(msg):
    #Define global varaibles
    global x_pose
    global y_pose
    #Read the values
    x_pose = msg.pose.pose.position.x
    y_pose = msg.pose.pose.position.y

def main():
    
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
    	rospy.init_node("robot_controller", anonymous="True",disable_signals=True)
     	main()	
	rate.sleep()
    except KeyboardInterrupt:
	print("Fuori")
