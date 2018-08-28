import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

count = 0

# Arbitrary starting point values
getXpix = 360
getYpix = 270
getZ = 0.535
getY = 0.66

def publisher():
    global count
    global getXpix
    global getYpix
    global getY
    global getZ

    # Initialising objects used to communicate coordinates to KUKA iiwa controller
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    joint = PoseStamped()

    # Arbitrary starting point values
    joint.header.seq = count
    joint.header.stamp = rospy.Time.now()
    joint.header.frame_id = ""
    joint.pose.position.x = -0.126
    joint.pose.orientation.x = 0.37997234165
    joint.pose.orientation.y = -0.596354351601
    joint.pose.orientation.z = -0.379993107133
    joint.pose.orientation.w = 0.596311748028

    # ball to left of the center but to the right of left boundary
    if ((getXpix < 360 - 15) and (getY - 0.010 >= 0.515)):
        # move left
        joint.pose.position.y = getY - 0.010

        # ball lower than center of the screen but higher than low boundary
        if ((getYpix > 270 + 15) and (getZ - 0.010 >= 0.140)):
            # move down
            joint.pose.position.z = getZ - 0.010

        # ball higher than center of the screen but lower than high boundary
        elif ((getYpix < 270 - 15) and (getZ + 0.010 <= 0.624)):
            # move up
            joint.pose.position.z = getZ + 0.010
        else:
            # keep position
            joint.pose.position.z = getZ

    # ball to right of the center but to the left of right boundary
    elif ((getXpix > 360 + 15) and (getY + 0.010 <= 0.774)):
        # move right
        joint.pose.position.y = getY + 0.010

        # ball lower than center of the screen but higher than low boundary
        if ((getYpix > 270 + 15) and (getZ - 0.010 >= 0.140)):
            # move down
            joint.pose.position.z = getZ - 0.010

        # ball higher than center of the screen but lower than high boundary
        elif ((getYpix < 270 - 15) and (getZ + 0.010 <= 0.624)):
            # move up
            joint.pose.position.z = getZ + 0.010

        else:
            # keep position
            joint.pose.position.z = getZ

    else:
        # keep position
        joint.pose.position.y = getY

        # ball lower than center of the screen but higher than low boundary
        if ((getYpix > 270 + 15) and (getZ - 0.010 >= 0.140)):
            # move down
            joint.pose.position.z = getZ - 0.010
        
        # ball higher than center of the screen but lower than high boundary
        elif ((getYpix < 270 - 15) and (getZ + 0.010 <= 0.624)):
            # move up
            joint.pose.position.z = getZ + 0.010

        else:
            # keep position
            joint.pose.position.z = getZ

    # Publishing the coordinates to KUKA iiwa controller
    rospy.loginfo(joint)
    pub.publish(joint)
    count += 1

def callback(data):
    global getYpix
    global count
    # Recording and printing ball position published by computer vision software
    rospy.loginfo('callback1: %f', data.data)
    getYpix = data.data

def callback2(data):
    global getXpix
    # Recording and printing ball position published by computer vision software
    rospy.loginfo('callback2: %f', data.data)
    getXpix = data.data

def callback3(data):
    global getZ
    # Recording and printing current robot position program
    rospy.loginfo('callback3: %f', data.data)
    getZ = data.data

def callback4(data):
    global getY
    # Recording and printing current robot position program
    rospy.loginfo('callback4: %f', data.data)
    getY = data.data
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

def listener():
    # Initialising node
    rospy.init_node('subscribertracking', anonymous=True)
    # Subscribing to the ball position as tracked by camera on robot's gripper
    rospy.Subscriber("centery", Float64, callback)
    rospy.Subscriber("centerx", Float64, callback2)
    # Subcribing to current robot position cordinates
    rospy.Subscriber("feedz", Float64, callback3)
    rospy.Subscriber("feedy", Float64, callback4)
    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
