#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import numpy as np
import math

# min_d = 0.6

# class wall_follower:
#     def __init__(self):
#         self.vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
#         self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laserCallBack)
#         self.regions = {}
#         while True:
#             self.take_action()

#     def	take_action(self):
#         if not self.regions:
#             return
#         linear_x	=	0
#         angular_z	=	0
#         state_description	=	''
#         msg = Twist()
#         if	(self.regions['front'] > min_d and	
#              self.regions['left'] > min_d and	
#              self.regions['right'] > min_d):
#             state_description	=	'case	1	- nothing (MOVING FORWARD)'
#             linear_x	=	0.3
#             angular_z	=	0
#         elif self.regions['front'] < min_d or np.isnan(self.regions['front']):
#             state_description	=	'case	2	- front (ALIGNING ROBOT TO WALL)'
#             linear_x	=	0
#             angular_z	=	0.3
#             prev_front = self.regions['front']
#             prev_right = self.regions['right']
#             while(self.regions['right'] > prev_front + 0.05 and prev_right != self.regions['right']):
#                 vel_pub.publish(msg)
#                 prev_right = self.regions['right']
#         elif (self.regions['front'] > min_d and	
#               self.regions['left'] > min_d and	
#               (self.regions['right'] < min_d or np.isnan(self.regions['right']))):
#             state_description	=	'case	3	- right (MOVING FORWARD AND FOLLOWING RIGHT SIDE)'
#             linear_x	=	0.3
#             angular_z	=	0
#         elif (regions['front'] > min_d and	
#               (regions['left'] < min_d or np.isnan(regions['left'])) and 
#               regions['right'] > min_d):
#             state_description	=	'case	4	- left'
#             linear_x	=	0
#             angular_z	=	-0.3
#         elif ((regions['front'] < min_d or np.isnan(regions['front'])) and	
#               regions['left'] > min_d and 
#               (regions['right'] < min_d or np.isnan(regions['right']))):
#             state_description	=	'case	5	- front	and	right'
#             linear_x	=	0
#             angular_z	=	0.3
#         elif ((regions['front'] < min_d or np.isnan(regions['front'])) and	
#               (regions['left'] < min_d or np.isnan(regions['left'])) and 
#               regions['right'] > min_d):
#             state_description	=	'case	6	- front	and	left'
#             linear_x	=	0
#             angular_z	=	-0.3
#         elif ((regions['front'] < min_d or np.isnan(regions['front'])) and	
#               (regions['left'] < min_d or np.isnan(regions['left'])) and 
#               (regions['right'] < min_d or np.isnan(regions['right']))):
#             state_description	=	'case	7	- front	and	fleft	and	right'
#             linear_x	=	0
#             angular_z	=	0.3
#         elif (regions['front'] > min_d and	
#               (regions['left'] < min_d or np.isnan(regions['left'])) and 
#               (regions['right'] < min_d or np.isnan(regions['right']))):
#             state_description	=	'case	8	- left	and	right'
#             linear_x	=	0.3
#             angular_z	=	0
#         else:
#             print(self.regions)
#             state_description	=	'unknown	case'

#         rospy.loginfo(state_description)
#         msg.linear.x	=   linear_x
#         msg.angular.z	=	angular_z
#         self.vel_pub.publish(msg)

#     def laserCallBack(self, laser_msg):
#         num = len(laser_msg.ranges) / 5
#         self.regions = {
#             'right': min(min(laser_msg.ranges[0:num]),	10),
#             # 'fright': min(min(laser_msg.ranges[num+1:2*num]),	10),
#             'front': min(min(laser_msg.ranges[2*num+1:3*num]),	10),
#             # 'fleft': min(min(laser_msg.ranges[3*num+1:4*num]),	10),
#             'left': min(min(laser_msg.ranges[4*num+1:]),	10),
#         }

# def main():
#     rospy.init_node('dumb_alg', disable_signals=True)
#     wf = wall_follower()
#     rospy.spin()

# if __name__ == '__main__':
#     main()

pub_ = None
regions_ = {
    'right': 0,
    'front': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}
	
def clbk_laser(msg):
    global regions_
    print(len(msg.ranges))
    regions_ = {
        'right':  min(min(msg.ranges[0:213]), 10),
        'front':  min(min(msg.ranges[213:426]), 10),
        'left':   min(min(msg.ranges[426:641]), 10),
    }
    print(regions_)
    take_action()
 
 
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state
	
def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 0.6
    
    if regions['front'] > d and regions['left'] > d and regions['right'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif (regions['front'] < d or np.isnan(regions['front'])) and regions['left'] > d and regions['right'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['left'] > d and (regions['right'] < d or np.isnan(regions['right'])):
        state_description = 'case 3 - right'
        change_state(2)
    elif regions['front'] > d and (regions['left'] < d or np.isnan(regions['left'])) and regions['right'] > d:
        state_description = 'case 4 - left'
        change_state(0)
    elif (regions['front'] < d or np.isnan(regions['front'])) and regions['left'] > d and (regions['right'] < d or np.isnan(regions['right'])):
        state_description = 'case 5 - front and right'
        change_state(1)
    elif (regions['front'] < d or np.isnan(regions['front'])) and (regions['left'] < d or np.isnan(regions['left'])) and regions['right'] > d:
        state_description = 'case 6 - front and left'
        change_state(1)
    elif (regions['front'] < d or np.isnan(regions['front'])) and (regions['left'] < d or np.isnan(regions['left'])) and (regions['right'] < d or np.isnan(regions['right'])):
        state_description = 'case 7 - front and left and right'
        change_state(1)
    elif regions['front'] > d and (regions['left'] < d or np.isnan(regions['left'])) and (regions['right'] < d or np.isnan(regions['right'])):
        state_description = 'case 8 - left and right'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
	
def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.3
    return msg
 
def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg
 
def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def main():
    global pub_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
    
    sub = rospy.Subscriber('scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()
 
if __name__ == '__main__':
    main()