import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

count = 0

# Arbitrary starting point values
getY = 0.66
getZ = 0.535

def publisher1():
    global count
    global getY
    global getZ

    # Initialising objects used to communicate coordinates to KUKA iiwa controller
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    joint = PoseStamped()

    # Preparing data for publishing to KUKA iiwa controller
    setZ = getZ
    setY = getY

    # Arbitrary starting point values
    joint.header.seq = count
    joint.header.stamp = rospy.Time.now()
    joint.header.frame_id = ""

    joint.pose.position.x = -0.126
    joint.pose.position.z = setZ #0.535
    joint.pose.position.y = setY #0.66
    joint.pose.orientation.x = 0.37997234165
    joint.pose.orientation.y = -0.596354351601
    joint.pose.orientation.z = -0.379993107133
    joint.pose.orientation.w = 0.596311748028

    # Publishing the coordinates to KUKA iiwa controller
    rospy.loginfo(joint)
    pub.publish(joint)
    count += 1

def callback(data):
    global count
    global getZ
    # Recording and printing ball catching position published by computer vision software
    rospy.loginfo('callback1: %f', data.data)
    getZ = data.data

def callback2(data):
    global getY
    # Recording and printing ball catching position published by computer vision software
    rospy.loginfo('callback2: %f', data.data)
    getY = data.data
    try:
        publisher1()
    except rospy.ROSInterruptException:
        pass

def listener():
    # Initialising node
    rospy.init_node('listener', anonymous=True)
    # Subscribing to the ball catching position as predicted by computer vision software
    rospy.Subscriber("positionZ", Float64, callback)
    rospy.Subscriber("positionY", Float64, callback2)
    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
