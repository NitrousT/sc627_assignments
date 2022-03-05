#!/usr/bin/env python3

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import *

#import other helper files if any


rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file
x_start = None
y_start = None
x_goal = None
y_goal = None
step_size = None

lines = []
with open('/home/danish/syscon_ws/src/sc627_assignments/assignment_1/input.txt', 'r') as file:
    lines = file.readlines()
        
start_coords = lines[0].strip()
x_start = int(start_coords[0])
y_start = int(start_coords[2])

goal_coords = lines[1].strip()
x_goal = int(goal_coords[0])
y_goal = int(goal_coords[2])

step_size = float(lines[2].strip())

obstaclesList = []
obstacle = []
for line in lines[4:]:
    if line=="\n":
        obstaclesList.append(obstacle)
        obstacle = []
    else:
        lin = line.split(',')
        lin = np.float32(lin)
        obstacle.append([lin[0],lin[1]])
obstaclesList.append(obstacle)

# output_file = open('output.txt', 'w')
# L = [x_start,y_start]
# output_file.writelines(L)
# output_file.close()

#setting result as initial location
current_x = x_start
current_y = y_start
 #in radians (0 to 2pi)           HAVE TO TAKE A LOOK!!!!
n_obstacles = len(obstaclesList)

# output_file = open('output.txt', 'a')
# print(wp.pose_dest.theta,0)

net_grad_x = None
net_grad_y = None

bot_dist = np.sqrt((x_goal-x_start)**2 + (y_goal - y_start)**2)

while (net_grad_x != 0. and net_grad_y != 0.):
    
    wp = MoveXYGoal()
    calculated_x = None
    calculated_y = None
    calculated_theta = None

    #determine waypoint based on your algo

    # r_grad_x, r_grad_y = replusive_pot(current_x, current_y, obstaclesList, 0.8, 2)
    a_grad_x, a_grad_y = attract_pot(current_x, current_y, x_goal, y_goal, 0.8, 2)

    r_grad_x, r_grad_y = repulsive_pot(current_x, current_y, obstaclesList, 0.8, 2)
    
    net_grad_x = a_grad_x + r_grad_x
    net_grad_y = a_grad_y + r_grad_y

    calculated_x = current_x -step_size*net_grad_x
    calculated_y = current_y -step_size*net_grad_y
    
    calculated_theta = np.arctan2(calculated_y, calculated_x)
    
    wp.pose_dest.x = calculated_x
    wp.pose_dest.y = calculated_y
    wp.pose_dest.theta = calculated_theta
    print(calculated_x, calculated_y, calculated_theta)
    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)

    client.wait_for_result()

    #getting updated robot location
    result = client.get_result()
    
    current_x = result.pose_final.x
    current_y = result.pose_final.y
    
    if np.sqrt((current_x-x_goal)**2 + (current_y-y_goal)**2) <= 0.001:
        break
    
    # bot_dist = np.sqrt((current_x-x_goal)**2 + (current_y-y_goal)**2)
    
    
    # if np.sqrt((current_x-x_goal)**2 + (current_y-y_goal)**2) < 0.1:
    #     break

    # print((current_x-x_goal), (current_y-y_goal))

    #write to output file (replacing the part below)
    # print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)