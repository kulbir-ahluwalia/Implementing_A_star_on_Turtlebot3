#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_bot():
	rospy.init_node("Driver", anonymous= True)
	drive = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	msg = Twist()

	lin_x = 0.2
	ang_z = -0.1
	distance = 2.5

	msg.linear.x = lin_x
	msg.angular.z = 0
	current_dist = 0

	while not rospy.is_shutdown():
		t0 = rospy.Time.now().to_sec()
		
		while current_dist<distance:
			drive.publish(msg)

			t1 = rospy.Time.now().to_sec()
			current_dist = lin_x * (t1-t0)
		msg.linear.x = 0
		rospy.spin()


if __name__ == '__main__':
	try:
		move_bot()
	except rospy.ROSInterruptException:
		pass

