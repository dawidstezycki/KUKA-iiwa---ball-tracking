import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

def publisher():
    global getZ
    # Sending coordinates to subscriber node
    pub = rospy.Publisher('feedz', Float64, queue_size=10)
    rospy.loginfo(getZ)
    pub.publish(getZ)

def callback(data):
    global getZ
    getZ = 0.535 # Arbitrary starting point value

    # Recording current robot position program
    getZ = data.pose.position.z
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

def listener():
    # Initialising node
    rospy.init_node('feedbackZ', anonymous=True)
    # Subscribing to current robot position coordinate Z
    rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback)
    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
