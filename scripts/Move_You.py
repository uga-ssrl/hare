#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def Move_You():
    #get namespace and append to cmd_vel
    pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    rospy.init_node('Move_You', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    global msg
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    while True:
        input = raw_input("Please enter your command as either N, S, E, W, or P to stop the robot. Any other key will exit the program.\n")
        if input == "N":
            msg.linear.x = 1
        elif input == "S":
            msg.linear.x = -1
        elif input == "E":
            msg.linear.y = 1
        elif input == "W":
            msg.linear.y = -1
        elif input == "P":
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
        else:
            break;

        rospy.loginfo(msg)
        pub.publish(msg)
        print("just published\n")
        #Sleeping for 1 second and resetting the msg contents to stop the robot
        time.sleep(1)
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        rospy.loginfo(msg)
        pub.publish(msg)

    #while not rospy.is_shutdown():
        #take user input and construct a message from it


if __name__ == '__main__':
    try:
        Move_You()
    except rospy.ROSInterruptException:
        pass
