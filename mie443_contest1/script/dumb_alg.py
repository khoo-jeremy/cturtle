#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

msg = Twist()

def	take_action(regions):
    linear_x	=	0
    angular_z	=	0
    state_description	=	''
    if	regions['front']	>	1	and	regions['fleft']	>	1	and	regions['fright']	>	1:
        state_description	=	'case	1	- nothing'
        linear_x	=	0.6
        angular_z	=	0
    elif	regions['front']	<	1	and	regions['fleft']	>	1	and	regions['fright']	>	1:
        state_description	=	'case	2	- front'
        linear_x	=	0
        angular_z	=	0.3
    elif	regions['front']	>	1	and	regions['fleft']	>	1	and	regions['fright']	<	1:
        state_description	=	'case	3	- fright'
        linear_x	=	0
        angular_z	=	0.3
    elif	regions['front']	>	1	and	regions['fleft']	<	1	and	regions['fright']	>	1:
        state_description	=	'case	4	- fleft'
        linear_x	=	0
        angular_z	=	-0.3
    elif	regions['front']	<	1	and	regions['fleft']	>	1	and	regions['fright']	<	1:
        state_description	=	'case	5	- front	and	fright'
        linear_x	=	0
        angular_z	=	0.3
    elif	regions['front']	<	1	and	regions['fleft']	<	1	and	regions['fright']	>	1:
        state_description	=	'case	6	- front	and	fleft'
        linear_x	=	0
        angular_z	=	-0.3
    elif	regions['front']	<	1	and	regions['fleft']	<	1	and	regions['fright']	<	1:
        state_description	=	'case	7	- front	and	fleft	and	fright'
        linear_x	=	0
        angular_z	=	0.3
    elif	regions['front']	>	1	and	regions['fleft']	<	1	and	regions['fright']	<	1:
        state_description	=	'case	8	- fleft	and	fright'
        linear_x	=	0.3
        angular_z	=	0
    else:
        state_description	=	'unknown	case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x	=	-linear_x
    msg.angular.z	=	angular_z

def laserCallBack(laser_msg):
    num = len(laser_msg.ranges) / 5
    regions	=	{
        'right': min(min(laser_msg.ranges[0:num]),	10),
        'fright': min(min(laser_msg.ranges[num+1:2*num]),	10),
        'front': min(min(laser_msg.ranges[2*num+1:3*num]),	10),
        'fleft': min(min(laser_msg.ranges[3*num+1:4*num]),	10),
        'left': min(min(laser_msg.ranges[4*num+1:]),	10),
    }
    take_action(regions)

def main():
    rospy.init_node('dumb_alg', disable_signals=True)
    rate = rospy.Rate(10) # 10hz
    vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
    laser_sub = rospy.Subscriber("scan", LaserScan, laserCallBack)
    while not rospy.is_shutdown():
        vel_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()