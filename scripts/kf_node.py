#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from ros_kalman_filter import kalman_filter
from kuka_kickass_kalman.msg import Obs


def callback(vel_msg):
    #once velocity is recieved, wait for next sensor reading
    if vel_msg.linear.x != 0 or vel_msg.linear.y != 0:
        time.sleep(1)
        sensor_msg = rospy.wait_for_message('sensor_readings', Obs)

        #housekeeping with the msgs
        ui     = vel_msg.linear.x
        uj     = vel_msg.linear.y
        uTruei = vel_msg.linear.x
        uTruej = vel_msg.linear.y
        zi     = sensor_msg.z1
        zj     = sensor_msg.z2
        zTruei = zi
        zTruej = zj

        u      = [ui,uj]         #noisy velocity
        uTrue  = [uTruei,uTruej] #true velocity
        z      = [zi,zj]         #noisy position
        zTrue  = [zTruei,zTruej] #true position

        #estimate the postition
        xEsti, xEstj = kf.move_forward(u,uTrue,z,zTrue,generate_input_noise = True,generate_measurement_noise = False)

        #caculate error 
        xErri = zTruei-xEsti
        xErrj = zTruej-xEstj

        #create and send position estimate messege
        global xEst_msg
        xEst_msg    = Pose()
        xEst_msg.position.x    = xEsti
        xEst_msg.position.y    = xEstj
        xEst_msg.position.z    = 0
        xEst_msg.orientation.x = 0
        xEst_msg.orientation.y = 0
        xEst_msg.orientation.z = 0
        xEst_msg.orientation.w = 0
        xEst_pub.publish(xEst_msg)

        #create and send error messege
        global xErr_msg
        xErr_msg    = Pose()
        xErr_msg.position.x    = xErri
        xErr_msg.position.y    = xErrj
        xErr_msg.position.z    = 0
        xErr_msg.orientation.x = 0
        xErr_msg.orientation.y = 0
        xErr_msg.orientation.z = 0
        xErr_msg.orientation.w = 0
        xErr_pub.publish(xErr_msg)
        
        #uncomment for testing
        #kf.graph()
        
        #log info
        rospy.loginfo("\nxEst"+str(xEst_msg)+"\neErr:"+str(xErr_msg))

def kf_node():
    global kf
    kf = kalman_filter()
    rospy.init_node('kf_node', anonymous=True)
    
    global xEst_pub
    global xErr_pub
    xEst_pub = rospy.Publisher('geometry_msgs/Pos/xEst', Pose, queue_size=10)
    xErr_pub = rospy.Publisher('geometry_msgs/Pos/error', Pose, queue_size=10)
    
    rospy.Subscriber('/cmd_vel', Twist, callback=callback)
    rospy.spin()

if __name__ == '__main__':
    kf_node()
