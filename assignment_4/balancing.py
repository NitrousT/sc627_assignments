#!/usr/bin/env python3

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import time
import numpy as np

ANG_MAX = math.pi
VEL_MAX = 0.15
k=0.5

bot = np.array([0, 0, 0])
left_bot = np.array([0, 0, 0])
right_bot = np.array([0, 0, 0])

def velocity_convert(theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 1 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

def callback_odom(data):
    '''
    Get robot data
    '''
    bot_pose_x = data.pose.pose.position.x
    bot_pose_y = data.pose.pose.position.y
    (_,_,bot_yaw) = euler_from_quaternion ([data.pose.pose.orientation.x,data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    global bot
    bot = [bot_pose_x, bot_pose_y, bot_yaw]

def callback_left_odom(data):
    '''
    Get left robot data
    '''
    print('left robot')
    left_bot_pose_x = data.pose.pose.position.x
    left_bot_pose_y = data.pose.pose.position.y
    (_,_,left_bot_yaw) = euler_from_quaternion ([data.pose.pose.orientation.x,data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    global left_bot
    left_bot = [left_bot_pose_x, left_bot_pose_y, left_bot_yaw]
    


def callback_right_odom(data):
    '''
    Get right robot data
    '''
    print('right robot')
    right_bot_pose_x = data.pose.pose.position.x
    right_bot_pose_y = data.pose.pose.position.y
    (_,_,right_bot_yaw) = euler_from_quaternion ([data.pose.pose.orientation.x,data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    global right_bot
    right_bot = [right_bot_pose_x, right_bot_pose_y, right_bot_yaw]


rospy.init_node('balancing', anonymous = True)
rospy.Subscriber('/odom', Odometry, callback_odom) #topic name fixed
rospy.Subscriber('/left_odom', Odometry, callback_left_odom) #topic name fixed
rospy.Subscriber('/right_odom', Odometry, callback_right_odom) #topic name fixed


pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
time.sleep(1)
r = rospy.Rate(30)

while True: #replace with balancing reached?
    #calculate v_x, v_y as per the balancing strategy
    v_x=k*((left_bot[0]-bot[0])+(right_bot[0]-bot[0]))
    v_y=k*((left_bot[1]-bot[1])+(right_bot[1]-bot[1]))
    #Make sure your velocity vector is feasible (magnitude and direction)
    # angle = bot[2]*180/np.pi
    # if angle < 0:
    #     angle += 360
        
    v_lin,v_ang=velocity_convert(bot[2], v_x, 0.0)
    #convert velocity vector to linear and angular velocties using velocity_convert function given above

    #publish the velocities below
    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = v_ang
    pub_vel.publish(vel_msg)
    
    #store robot path with time stamps (data available in odom topic)

    r.sleep()




