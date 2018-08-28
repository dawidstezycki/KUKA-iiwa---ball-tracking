#!/usr/bin/env python
import rospy
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity
from std_msgs.msg import Header

def talker():
	# Initialising objects used to communicate coordinates to KUKA iiwa controller
	pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
	joint = JointPosition()

	# Initialising node
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)  # 10hz sending frequency
	count = 0

	while not rospy.is_shutdown():
		joint.header.seq = count
		joint.header.stamp = rospy.Time.now()
		joint.header.frame_id = ""

		# Zeroing all the joints' coordinates - robotic arm stretches upwards
		joint.position.a1 = 0
		joint.position.a2 = 0
		joint.position.a3 = 0
		joint.position.a4 = 0
		joint.position.a5 = 0
		joint.position.a6 = 0
		joint.position.a7 = 0

		# Publishing the coordinates to KUKA iiwa controller
		rospy.loginfo(joint)
		pub.publish(joint)
		rate.sleep()
		count += 1
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
