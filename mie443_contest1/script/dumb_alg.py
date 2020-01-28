import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class motion_planning:

	def __init__(self):
		self.vel_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
    	self.laser_sub = rospy.Subscriber("scan", 10, self.laserCallBack)
	
	def	take_action(self, regions):
		msg	=	Twist()
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
		self.vel_pub.publish(msg)

	def laserCallBack(self, msg):
		regions	=	{
			'right': min(min(msg.ranges[0:100]),	10),
			'fright': min(min(msg.ranges[101:200]),	10),
			'front': min(min(msg.ranges[201:300]),	10),
			'fleft': min(min(msg.ranges[301:400]),	10),
			'left': min(min(msg.ranges[401:500]),	10),
		}
		self.take_action(regions)

def main():
    rospy.init_node('dumb_alg', disable_signals=True)
    motion_plan = motion_planning()
    rospy.spin()