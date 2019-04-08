#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

class Neighbor:
    id = -1
    info = {}

class Robot:
    id = -1
    info = {}
    neighbors = []


    def __init__(self, id):
        self.id = id
        initializeNeighbors()

    #this needs to read params that are set  in launch files
    def initializeNeighbors(self):
        return True


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def rcvData():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
def sendData():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()



def main():
    return 0



if __name__ == '__main__':
    main()
