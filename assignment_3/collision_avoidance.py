#!/usr/bin/env python3

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from anti_feasible_angle import *
from collision_vec import *
import time

ANG_MAX = math.pi/18
VEL_MAX = 0.15
bot_pose = [0,0]

def velocity_convert(theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 1 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    # if ang < 0:                                       # This is done to stop the bot spinning unnecessarily
    #     ang += 2 * math.pi
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

obstacle_info = {"bot_2":[], "bot_3":[], "bot_4":[]}    # dictionary to store obstacle data


def callback_obs(data):
    '''
    Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
    '''
    obstacle_info[data.obstacles[0].obs] = [data.obstacles[0].pose_x, data.obstacles[0].pose_y, data.obstacles[0].vel_x, data.obstacles[0].vel_y]
    obstacle_info[data.obstacles[1].obs] = [data.obstacles[1].pose_x, data.obstacles[1].pose_y, data.obstacles[1].vel_x, data.obstacles[1].vel_y]
    obstacle_info[data.obstacles[2].obs] = [data.obstacles[2].pose_x, data.obstacles[2].pose_y, data.obstacles[2].vel_x, data.obstacles[2].vel_y] 
    
    pass



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
time.sleep(3)

pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 15)
r = rospy.Rate(30)


goal = [5,0]                    # Goal location
bot_rad = 0.075                 # Bot's radius
obs_rad= 0.075                  # Obstacles's radius


while np.sqrt((bot_pose[0]-goal[0])**2+(bot_pose[1]-goal[1])**2)>=0.1:
    print("<<------------------EXECUTING------------------>>")
    #calculate collision cone below
    new_obs_rad = bot_rad+obs_rad
    obj_vec = np.array([(goal[0]-bot_pose[0]),(-bot_pose[1]+goal[1])])/(np.sqrt((bot_pose[0]-goal[0])**2 + (bot_pose[1]-goal[1])**2))
    obj_angle = np.round(nice_angle(math.atan2(obj_vec[1], obj_vec[0]))*180/np.pi)                                                      # Angle from Bot's location to the goal

    # Bot's location from all the obstacles
    dist_obs2_bot = np.sqrt((bot_pose[0]-obstacle_info['bot_2'][0])**2+(bot_pose[1]-obstacle_info['bot_2'][1])**2)                      
    dist_obs3_bot = np.sqrt((bot_pose[0]-obstacle_info['bot_3'][0])**2+(bot_pose[1]-obstacle_info['bot_3'][1])**2)
    dist_obs4_bot = np.sqrt((bot_pose[0]-obstacle_info['bot_4'][0])**2+(bot_pose[1]-obstacle_info['bot_4'][1])**2)
    # Tangent vectors of the cone 
    cone_1_up,cone_1_low = colcone_vec(bot_pose, [obstacle_info['bot_2'][0],obstacle_info['bot_2'][1]], new_obs_rad)
    cone_2_up,cone_2_low = colcone_vec(bot_pose, [obstacle_info['bot_3'][0],obstacle_info['bot_3'][1]], new_obs_rad)
    cone_3_up,cone_3_low = colcone_vec(bot_pose, [obstacle_info['bot_4'][0],obstacle_info['bot_4'][1]], new_obs_rad)
    # The collision cone
    cone_1 = col_cone(cone_1_low, cone_1_up,)
    cone_2 = col_cone(cone_2_low, cone_2_up,)
    cone_3 = col_cone(cone_3_low, cone_3_up,)
    
    all_angles = np.arange(0,360,1)
    feasible_angles = set(all_angles) - set(cone_3).union(set(cone_1),set(cone_2))
    
    feasible_angles = np.array(list(feasible_angles))                                           # Set of feasible angles the bot may take to avoid all the obstacles
    
    if len(feasible_angles) == 0:                                                               # Boundary case when there are no feasible angles 
        chosen_angle = (obj_angle)
        cmd_vel = [0,0]
        
    else:
        chosen_angle = feasible_angles[np.argmin(np.absolute(feasible_angles-obj_angle))]       # Angle closest to the angle that takes us to the goal
        
    # print(cone_1, cone_2, cone_3)
    
    cmd_vel = [0.15*np.cos((chosen_angle)*np.pi/180), 0.15*np.sin((chosen_angle)*np.pi/180)]    # Velocity components
    v_lin, v_ang = velocity_convert((yaw), cmd_vel[0], cmd_vel[1])                              # Conversion of velocity to linear and angular using the given function
    time.sleep(2)
    
    if np.sqrt((bot_pose[0]-goal[0])**2+(bot_pose[1]-goal[1])**2)<=0.1:                         # To stop the Bot when goal is reached
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            pub_vel.publish(vel_msg)
            break
        
    #publish the velocities
    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = v_ang
    pub_vel.publish(vel_msg)
    

    r.sleep()




