import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

def publisher():
    global getY
    # Sending coordinates to subscriber node
    pub = rospy.Publisher('feedy', Float64, queue_size=10)
    rospy.loginfo(getY)
    pub.publish(getY)

def callback(data):
    global getY
    getY = 0.66 # Arbitrary starting point value

    # Recording current robot position program
    getY = data.pose.position.y

    # Printing the coordinates in terminal
    rospy.loginfo(data.pose.position.y)
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

def listener():
    # Initialising node
    rospy.init_node('feedbackY', anonymous=True)
    # Subscribing to current robot position coordinate Y
    rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback)
    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
