#! /usr/bin/env python

import rospy
import time
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from final_assignment.srv import Target

import math

pub = None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
srv_client_user_interface_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None

move_base_goal = MoveBaseGoal()
move_base_goal.target_pose.pose.position.x = rospy.get_param('des_pos_x')
move_base_goal.target_pose.pose.position.y = rospy.get_param('des_pos_y')
move_base_goal.target_pose.pose.orientation.w = 1.0
move_base_goal.target_pose.header.frame_id = '/map'



state_desc_ = ['Go to point', 'Wall following for avoiding obstacles', 'Mission accomplished! :)', 'I love following the walls.','I am sleeping.','Planning algorithm changed!']
state_ = rospy.get_param('state_value')
# 0 - go to point via move base
# 1 - wall following for bug_0
# 2 - mission compltere
# 3 - looking for closer wall and wall following 
# 4 - sleeping in the position

def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


def change_state(state):
    global state_, state_desc_, start, bug_trigger
    global srv_client_wall_follower_, srv_client_go_to_point
    

    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    
    pub_canc = rospy.Publisher('/move_base/cancel', GoalID, queue_size = 10, latch = True)
   

    if state_ == 0:

	if bug_trigger == 1:
            resp = srv_client_go_to_point_(True)
            resp = srv_client_wall_follower_(False)
	else: 
            resp = srv_client_go_to_point_(False)
            resp = srv_client_wall_follower_(False)
	    move_base_goal.target_pose.pose.position.x = rospy.get_param('des_pos_x')
	    move_base_goal.target_pose.pose.position.y = rospy.get_param('des_pos_y') 
	    goal = MoveBaseActionGoal()
	    goal.goal = move_base_goal  
	    pub_move.publish(goal)

	# Read the new desired position
	desired_position_.x = rospy.get_param('des_pos_x')
	desired_position_.y = rospy.get_param('des_pos_y')

	
         
    elif state_ == 1:
	resp = srv_client_go_to_point_(False)
	resp = srv_client_wall_follower_(True)

    elif state_ == 2:

	#Cancel the target for move base
        goalid = GoalID()
	pub_canc.publish(goalid)
	print("Target cancelled")

        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
	start = 1
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)        
        resp = srv_client_user_interface_()

    elif state_ == 3:
	
	resp = srv_client_wall_follower_(True)

    elif state_ == 4:
	pass


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_, start, bug_trigger
    global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_user_interface_, pub, pub_move


    rospy.init_node('main')
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_move = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 10, latch = True)
    
    
    srv_client_go_to_point_ = rospy.ServiceProxy(
        '/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    srv_client_user_interface_ = rospy.ServiceProxy('/user_interface', Empty)

    # initialize the change state trigger    
    start = 1

    # define the maximum time allowed to bug_0 to reach the target
    max_bug_time = 120.0 

    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():

	# read the state
	state_ = rospy.get_param('state_value')
	# read the active algorithm for reaching the goal
    	bug_trigger = rospy.get_param('bug_trigger')
	

	# update the state the first time that you pass here in the code
	if start == 1:
	    print("State changed:")
	    print(state_)
	    change_state(state_)
	    if bug_trigger == 1:
		start_time_bug = time.time()
	    start = 0	
	
        if regions_ == None:
            continue
 	
        if state_ == 0:
	    
            err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                    2) + pow(desired_position_.x - position_.x, 2))

	    #print(err_pos) 

	    # Change to move_base if the target has not been reached after 120 seconds using bug_0
	    if bug_trigger == 1: 

		delta_bug_time = time.time() - start_time_bug

		if delta_bug_time > max_bug_time:
		    print('Bug_0 has not been able to reach the target!')
		    print('Algorithm updated:')
		    resp = srv_client_wall_follower_(False)
		    resp = srv_client_go_to_point_(False)
		    delta_bug_time = 0.0
		    rospy.set_param('bug_trigger', 0)
		    start = 1
		
	    # trick not to get stuck close to the target  
            if(err_pos < 0.3):
		twist_msg = Twist()
        	twist_msg.linear.x = 0.1 * math.sqrt(pow(desired_position_.y - position_.y,
                                    2) + pow(desired_position_.x - position_.x, 2))
        	twist_msg.angular.z = 0
        	pub.publish(twist_msg) 
		change_flag = 1
                if change_flag == 1 and err_pos < 0.25:
		    twist_msg.linear.x = 0
        	    twist_msg.angular.z = 0
        	    pub.publish(twist_msg)
		    change_state(2)
		
		
            elif err_pos > 0.3 and regions_['front'] < 0.5 and bug_trigger == 1:
		change_state(1)

        elif state_ == 1:

	              	    	    	
            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                    2) + pow(desired_position_.x - position_.x, 2))

            
        elif state_ == 2:

	    move_base_goal.target_pose.pose.position.x = rospy.get_param('des_pos_x')
	    move_base_goal.target_pose.pose.position.y = rospy.get_param('des_pos_y')
            desired_position_.x = rospy.get_param('des_pos_x')
            desired_position_.y = rospy.get_param('des_pos_y')
            err_pos = math.sqrt(pow(desired_position_.y - position_.y,
                                    2) + pow(desired_position_.x - position_.x, 2))
	    
            if(err_pos > 0.35):
                change_state(0)

	elif state_ == 3:

	    #Use the robot position and the information about the closer wall todrive the robot close to it and start the wall following algorithm
	    twist_msg = Twist()
	    key_min = min(regions_.keys(), key=(lambda k : regions_[k]))
	    while key_min != 'front':
		#Turn the robot to go to the closer wall detected via the radar
        	twist_msg.angular.z = 1.0
        	pub.publish(twist_msg) 
	        key_min = min(regions_.keys(), key=(lambda k : regions_[k]))
		
	    #Set the angular velocity to zero and go close to the wall
	    twist_msg.linear.x = 0.2
	    twist_msg.angular.z = 0.0
            pub.publish(twist_msg) 		

	    if regions_['front'] < 0.3:

		print("Closer wall reached.")
		print("Wall fallowing started.")
		twist_msg.linear.x = 0.0
		pub.publish(twist_msg)
                   
		# follow the wall 
		start_wall_time = time.time()
	    	stop_wall_time = rospy.get_param('target_time')
	    	delta_wall_time = 0.0
	    	while delta_wall_time <  stop_wall_time:
		    delta_wall_time = time.time() - start_wall_time
	    	print("I have been following the walls for %s seconds" % delta_wall_time)
		# stop following the wall and declare the mission accomplished
		resp = srv_client_wall_follower_(False)
            	change_state(2)         	

        elif state_ == 4:

	    start_time = time.time()
	    stop_time = rospy.get_param('target_time')
	    delta_time = 0.0
	    while delta_time <  stop_time:
		delta_time = time.time() - start_time
	    print("I have been here for %s seconds" % delta_time)
            change_state(2)

	elif state_ == 5:
	    
            change_state(2)
   
        rate.sleep()

if __name__ == "__main__":
    main()
