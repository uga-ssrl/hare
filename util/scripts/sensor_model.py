#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from kuka_kickass_kalman.msg import Obs

def callback (msg):
	global sensor_pub
	if len(msg.pose) > 1: #The robot model pose needs to be present in the msg
		sigma = 0.2 #The std dev for the gaussian noise of the sensors
		obsmsg = Obs()
		xTrue = msg.pose[1].position.x
		yTrue = msg.pose[1].position.y
		xEst = xTrue + np.random.randn() * sigma #Generate the x position with Gaussian noise distribution
		yEst = yTrue + np.random.randn() * sigma #Assume y position affected by noise with same std dev
		obsmsg.z1 = xEst
		obsmsg.z2 = yEst
		sensor_pub.publish(obsmsg)
    

def sensor_model():
	global sensor_pub
    
	rospy.init_node('sensor_model', anonymous=True)
	rospy.Rate(10)
	rospy.Subscriber("/gazebo/model_states",ModelStates,callback) #Subscribe to ground truth to get true pose of robot

	sensor_pub = rospy.Publisher('sensor_readings', Obs, queue_size=10) #Publish on topic sensor_readings
	rospy.spin()

if __name__ == '__main__':
	sensor_model()


