#!/usr/bin/env python3

from os import minor
import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from feasible_angle import *

ANG_MAX = math.pi/18
VEL_MAX = 0.15

def velocity_convert(x, y, theta, vel_x, vel_y):
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

obstacle_info = {"bot_2":[], "bot_3":[], "bot_4":[]} # dictionary to store obstacle data


def callback_obs(data):
    '''
    Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
    '''
    # obs_data[data.obs]
    # if (data.obstacles[1].obs == 'bot_3'):
    #     print(1) 
    obstacle_info[data.obstacles[0].obs] = [data.obstacles[0].pose_x, data.obstacles[0].pose_y, data.obstacles[0].vel_x, data.obstacles[0].vel_y]
    obstacle_info[data.obstacles[1].obs] = [data.obstacles[1].pose_x, data.obstacles[1].pose_y, data.obstacles[1].vel_x, data.obstacles[1].vel_y]
    obstacle_info[data.obstacles[2].obs] = [data.obstacles[2].pose_x, data.obstacles[2].pose_y, data.obstacles[2].vel_x, data.obstacles[2].vel_y] 
    
    pass

bot_pose = [0,0]

def callback_odom(data):
    '''
    Get robot data
    '''
    global bot_pose 
    bot_pose = [data.pose.pose.position.x,data.pose.pose.position.y]
    global yaw
    (_,_,yaw) = euler_from_quaternion ([data.pose.pose.orientation.x,data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    pass


rospy.init_node('assign3_skeleton', anonymous = True)
rospy.Subscriber('/obs_data', ObsData, callback_obs) #topic name fixed
rospy.Subscriber('/bot_1/odom', Odometry, callback_odom) #topic name fixed
rospy.sleep(3)

pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
r = rospy.Rate(30)


goal = [5,0]                    # Goal location
bot_rad = 0.075                 # Bot's radius
obs_rad= 0.075                  # Obstacles's radius


while np.sqrt((bot_pose[0]-goal[0])**2+(bot_pose[1]-goal[1])**2)>=0.01:
    print("###########################################################")
    #calculate collision cone below
    new_obs_rad = bot_rad+obs_rad
    obj_vec = np.array([(goal[0]-bot_pose[0]),(-bot_pose[1]+goal[1])])/(np.sqrt((bot_pose[0]-goal[0])**2 + (bot_pose[1]-goal[1])**2))
    obj_angle = math.atan2(obj_vec[1], obj_vec[0])

    dist_obs2_bot = np.sqrt((bot_pose[0]-obstacle_info['bot_2'][0])**2+(bot_pose[1]-obstacle_info['bot_2'][1])**2)
    dist_obs3_bot = np.sqrt((bot_pose[0]-obstacle_info['bot_3'][0])**2+(bot_pose[1]-obstacle_info['bot_3'][1])**2)
    dist_obs4_bot = np.sqrt((bot_pose[0]-obstacle_info['bot_4'][0])**2+(bot_pose[1]-obstacle_info['bot_4'][1])**2)

    all_angles = np.arange(0,360,1)
    cone_1 = feasible_ang(new_obs_rad, bot_pose, [obstacle_info['bot_2'][0],obstacle_info['bot_2'][1]],[obstacle_info['bot_2'][2],obstacle_info['bot_2'][3]], 0.15, 360)
    cone_3 = feasible_ang(new_obs_rad, bot_pose, [obstacle_info['bot_4'][0],obstacle_info['bot_4'][1]],[obstacle_info['bot_4'][2],obstacle_info['bot_4'][3]], 0.15, 360)
    cone_2 = feasible_ang(new_obs_rad, bot_pose, [obstacle_info['bot_3'][0],obstacle_info['bot_3'][1]],[obstacle_info['bot_3'][2],obstacle_info['bot_3'][3]], 0.15, 360)
    print(cone_3)
    
    feasible_angles = set(all_angles) - set(cone_3)
    
    feasible_angles = np.array(list(feasible_angles))
    
    if len(feasible_angles) == 0:
        chosen_angle = 0
        cmd_vel = [0,0]
    else:
        chosen_angle = feasible_angles[np.argmin(np.absolute(feasible_angles-obj_angle))]
        cmd_vel = [0.15*np.cos((chosen_angle)*np.pi/180), 0.15*np.sin((chosen_angle)*np.pi/180)]
    
    # print('Best angle: ',chosen_angle)
    # print(np.sqrt(cmd_vel[0]**2+cmd_vel[1]**2), chosen_angle)
    
    
        
    v_lin, v_ang = velocity_convert(bot_pose[0], bot_pose[1], yaw*180/np.pi, cmd_vel[0], cmd_vel[1])
        

    #calculate v_x, v_y as per either TG, MV, or ST strategy
    #Make sure your velocity vector is feasible (magnitude and direction)

    #convert velocity vector to linear and angular velocties using velocity_convert function given above

    #publish the velocities below
    vel_msg = Twist()
    # vel_msg.linear.x = v_lin
    # vel_msg.angular.z = v_ang
    pub_vel.publish(vel_msg)
    
    #store robot path with time stamps (data available in odom topic)

    r.sleep()




