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
#result = MoveXYResult()
current_x = x_start
current_y = y_start
n_obstacles = len(obstaclesList)

# output_file = open('output.txt', 'a')
# print(wp.pose_dest.theta,0)
bot_dist = np.sqrt((x_goal-x_start)**2 + (y_goal - y_start)**2)

while (bot_dist) > step_size:
    
    wp = MoveXYGoal()
    calculated_x = None
    calculated_y = None
    calculated_theta = None

    #determine waypoint based on your algo
    obj_vec = np.array([(x_goal-current_x),(y_goal-current_y)])/(np.sqrt((x_goal-current_x)**2 + (y_goal-current_y)**2))
    min_distances = np.zeros(n_obstacles)
    
    for i in range(n_obstacles):
        out = np.array(computeDistancePointToPolygon(obstaclesList[i], current_x, current_y))
        min_distances[i] = min(out[:,0])
    
    
    if all(x > step_size for x in min_distances):
        
        calculated_x = current_x + step_size*obj_vec[0]
        calculated_y = current_y + step_size*obj_vec[1]
        calculated_theta = np.arctan2(obj_vec[1], obj_vec[0])
        wp.pose_dest.x = calculated_x
        wp.pose_dest.y = calculated_y
        wp.pose_dest.theta = calculated_theta
            #send waypoint to turtlebot3 via move_xy server
        client.send_goal(wp)

        client.wait_for_result()

        #getting updated robot location
        result = client.get_result()
    
        current_x = result.pose_final.x
        current_y = result.pose_final.y
        # print(wp.pose_dest.theta,0)
        # path.append([x_start,y_start])
        print(current_x,',',current_y)
    
    
    else:
        print('collision detected!')
        print('Collision Point: ', current_x," ",current_y)
        obstacle_no = np.argmin(min_distances)
        collision_x = current_x
        collision_y = current_y
        FollowObstacle = 1
        arr = []
        dist2obstacle_collide = min(computeDistancePointToPolygon(obstaclesList[obstacle_no],current_x, current_y))
        while FollowObstacle:
            new_direction = computeTangentVectorToPolygon(obstaclesList[obstacle_no], current_x, current_y)
            dist2obstacle = min(computeDistancePointToPolygon(obstaclesList[obstacle_no],current_x, current_y))
            mod_val = np.dot(new_direction,[1,0])
            # print(mod_val)
            if dist2obstacle[0] > dist2obstacle_collide[0]:
                if mod_val <=0.001 and mod_val >=-0.001:
                    current_x += (dist2obstacle[0]-dist2obstacle_collide[0])
                elif mod_val >= 0.999 and mod_val <= 1.001:
                    current_y += (dist2obstacle[0]-dist2obstacle_collide[0])


            calculated_x = current_x + step_size*new_direction[0]
            calculated_y = current_y + step_size*new_direction[1]
            calculated_theta = np.arctan2(new_direction[1],new_direction[0])
            
            wp.pose_dest.x = calculated_x
            wp.pose_dest.y = calculated_y
            wp.pose_dest.theta = calculated_theta
            
            # Sending the Waypoint
            client.send_goal(wp)
            client.wait_for_result()

            #getting updated robot location
            result = client.get_result()
            
            current_x = result.pose_final.x
            current_y = result.pose_final.y
            print(current_x,",",current_y)
            
            bot_dist = np.sqrt((current_x - x_goal)**2 + (current_y - y_goal)**2)
            arr.append([calculated_x, calculated_y, bot_dist])
            
            if np.sqrt((collision_x-current_x)**2 + (collision_y-current_y)**2) <0.05:
                print('Loop Break !')
                break
    
        arr = np.array(arr)
        min_distance2goal = min(arr[:,-1])
        req_index = np.argmin(arr[:,-1])
        
        if req_index == (len(arr) - 1):
            req_index = -1
        x_leave = arr[req_index,0]
        y_leave = arr[req_index,1]
        print('Leave Point: ',x_leave, y_leave)
       
        while FollowObstacle:
        
            new_direction = computeTangentVectorToPolygon(obstaclesList[obstacle_no], current_x, current_y)
            mod_val = np.dot(new_direction,[1,0])
            # print(mod_val)
            if dist2obstacle[0] > dist2obstacle_collide[0]:
                if mod_val <=0.001 and mod_val >=-0.001:
                    current_x += (dist2obstacle[0]-dist2obstacle_collide[0])
                elif mod_val >= 0.999 and mod_val <= 1.001:
                    current_y += (dist2obstacle[0]-dist2obstacle_collide[0])


        
            calculated_x = current_x + step_size*new_direction[0]
            calculated_y = current_y + step_size*new_direction[1]
            calculated_theta = np.arctan2(new_direction[1],new_direction[0])
            
            wp.pose_dest.x = calculated_x
            wp.pose_dest.y = calculated_y
            wp.pose_dest.theta = calculated_theta
            
            # Sending the Waypoint
            client.send_goal(wp)
            client.wait_for_result()

            #getting updated robot location
            result = client.get_result()
            
            current_x = result.pose_final.x
            current_y = result.pose_final.y
            print(current_x,",",current_y)

            if np.sqrt((x_leave-current_x)**2 + (y_leave-current_y)**2) <0.075:
                print('Loop Break !')
                break
           
        print('Found Min Point! Leaving...')
        FollowObstacle = 0
        new_mag = np.sqrt((x_goal-current_x)**2 +(y_goal-current_y)**2)
        new_direction = np.array([(x_goal-current_x)/new_mag, (y_goal-current_y)/new_mag])
        calculated_x = current_x +step_size*2*new_direction[0]
        calculated_y = current_y + step_size*2*new_direction[1]
        calculated_theta = calculated_theta = np.arctan2(new_direction[1],new_direction[0])
            
        wp.pose_dest.x = calculated_x
        wp.pose_dest.y = calculated_y
        wp.pose_dest.theta = calculated_theta
        
        # Sending the Waypoint
        client.send_goal(wp)
        client.wait_for_result()

        #getting updated robot location
        result = client.get_result()
        
        current_x = result.pose_final.x
        current_y = result.pose_final.y
        print(current_x,",",current_y)
        
        
            
        
    
    
    
    # if np.sqrt((current_x-x_goal)**2 + (current_y-y_goal)**2) < 0.1:
    #     break

    # print((current_x-x_goal), (current_y-y_goal))

    #write to output file (replacing the part below)
    # print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)