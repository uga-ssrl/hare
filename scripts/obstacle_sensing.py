#!/usr/bin/env python
import rospy
from hare.msg import Obstacle
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
# see: http://docs.ros.org/kinetic/api/nav_msgs/html/msg/Odometry.html
from std_msgs.msg import String, Float64

# ======= #
# GLOBALS #
# ======= #

# left   -> right
# bottom -> top
# [x1,x2,y1,y2],type
#
# Obstacles entered by type, from top to bottom
Obst_Info_DB = [
    ([   -6,   -4,   12, 12.5], 8), # Small tunnel ENTRANCE
    ([    1,    3,    9, 9.5 ], 8),
    ([   -6,   -4,  7.5, 8   ], 8),
    ([    9,  9.5,    3, 5   ], 8),
    ([   -7,   -5,    2, 2.5 ], 8),
    ([  1.5,    2,   -1,   1 ], 8),
    ([   10,  1.5,   -1,   1 ], 8),
    ([   -7,   -5,-10.5,  -11], 8),
    ([ -5.5,   -5,  -12,  -10], 8),
    ([   -1, -0.5,  -12,  -10], 8),
    ([-10.5,  -10,    6,    8], 9), # Large tunnel ENTRANCE
    ([   -6, -5.5,    6,    8], 9),
    ([  0.5,    1,    3,    5], 9),
    ([   -1,    1,   -2, -1.5], 9),
    ([   -1,    1,-11.5,  -10], 9),
    ([    3,  3.5,   -3,   -1], 10), # Ramp ENTRANCE
    ([  8.5,    9,   -3,   -1], 10),
    ([   -5,   -1, -2.5,   -2], 10),
    ([   -5,   -1,   -8, -7.5], 10),
    ([   -6,   -4, 11.5,   12], 11), # Small Tunnel EXIT
    ([   -6,   -4,    8,  8.5], 11),
    ([    1,    3,  8.5,    9], 11),
    ([   -7,   -5,  1.5,    2], 11),
    ([    2,  2.5,   -1,    1], 11),
    ([  9.5,   10,   -1,    1], 11),
    ([   -7,   -5,  -10, -9.5], 11),
    ([   -5, -4.5,  -12,  -10], 11),
    ([ -1.5,   -1,  -12,  -10], 11),
    ([  -10,-11.5,    6,    8], 12), # Large tunnel EXIT
    ([ -6.5,   -6,    6,    8], 12),
    ([    1,  1.5,    3,    5], 12),
    ([   -1,    1,  2.5,    2], 12),
    ([   -1,    1,   10, -9.5], 12),
    ([  3.5,    4,   -3,   -1], 13), # Ramp EXIT
    ([    8,  8.5,   -3,   -1], 13),
    ([   -1,   -5,   -3, -2.5], 13),
    ([ -7.5,   -7,   -3, -2.5], 13),
    ([  4.5,    5,    3,    5], 8), # MIXED
    ([  4.5,    5,    3,    5], 12), # MIXED
    ([    5,  5.5,    3,    5], 9), # MIXED
    ([    5,  5.5,    3,    5], 11), # MIXED
    ([   -7,   -5,   -2, -1.5], 11), # MIXED
    ([   -7,   -5,   -2, -1.5], 9), # MIXED
    ([   -7,   -5, -2.5,   -2], 12), # MIXED
    ([   -7,   -5, -2.5,   -2], 8), # MIXED
    ([   -7,   -5,   -6, -5.5], 11), # MIXED
    ([   -7,   -5,   -6, -5.5], 9), # MIXED
    ([   -7,   -5, -6.5,   -6], 12), # MIXED
    ([   -7,   -5, -6.5,   -6], 8) # MIXED
]

pub = []
sub = []
height =
width =

def getObstacleIds(x,y,z):
    dat_list = []
    tup = [14,14,(69,69,69),(69,69,69)] # default empty obstacles
    cells = a

    for obst in Obst_Info_DB:
        x_1 = obst[0][0]
        x_2 = obst[0][1] #
        y_1 = obst[0][2]
        y_2 = obst[0][3]
        draw_up = True if x_1 < x_2 else False
        draw_right = True if y_1 < y_2 else False

        while x_1 != x_2:
            if draw_up and down:

    return tup

# NOTE MUST ADD CALLBACK WHEN ADDING ROBOT
# Subscriber callback
def callback_robot1(msg): # callback which is called everytime there is a message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    # the x,y,z location of the bot
    print (x,y,z)
    tup = getObstacleIds(x,y,z)

    my_guy = Obstacle()
    my_guy.type1 = tup[0]
    my_guy.type2 = tup[1]

    my_guy.location1.x = tup[2][0]
    my_guy.location1.y = tup[2][1]
    my_guy.location1.z = tup[2][2]

    my_guy.location2.x = tup[3][0]
    my_guy.location2.y = tup[3][1]
    my_guy.location2.z = tup[3][2]

    pub[0].publish(my_guy)

def callback_robot2(msg): # callback which is called everytime there is a message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    # the x,y,z location of the bot
    print (x,y,z)
    tup = getObstacleIds(x,y,z)

    my_guy = Obstacle()
    my_guy.type1 = tup[0]
    my_guy.type2 = tup[1]

    my_guy.location1.x = tup[2][0]
    my_guy.location1.y = tup[2][1]
    my_guy.location1.z = tup[2][2]

    my_guy.location2.x = tup[3][0]
    my_guy.location2.y = tup[3][1]
    my_guy.location2.z = tup[3][2]

    pub[1].publish(my_guy)

def callback_robot3(msg): # callback which is called everytime there is a message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    # the x,y,z location of the bot

    tup = getObstacleIds(x,y,z)

    my_guy = Obstacle()
    my_guy.type1 = tup[0]
    my_guy.type2 = tup[1]

    my_guy.location1.x = tup[2][0]
    my_guy.location1.y = tup[2][1]
    my_guy.location1.z = tup[2][2]

    my_guy.location2.x = tup[3][0]
    my_guy.location2.y = tup[3][1]
    my_guy.location2.z = tup[3][2]

    pub[2].publish(my_guy)

# Publisher
# entry point in node
if __name__ == "__main__":
    rospy.init_node('obstacle_sensing')
    sub.append(rospy.Subscriber('/robot1/nav_msgs/odom', Odometry, callback_robot1))
    sub.append(rospy.Subscriber('/robot2/odom', Odometry, callback_robot2))
    # sub.append(rospy.Subscriber('/robot3/nav_msgs/odom', Odometry, callback_robot3))
    sub.append(rospy.Subscriber('/robot3/husky_velocity_controller/odom', Odometry, callback_robot3))
    pub.append(rospy.Publisher('/robot1/obstacle_sensing', Obstacle))
    pub.append(rospy.Publisher('/robot2/obstacle_sensing', Obstacle))
    pub.append(rospy.Publisher('/robot3/obstacle_sensing', Obstacle))
    rospy.spin()
