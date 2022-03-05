import numpy as np
from helper import *

def bug_base(start, goal, obstaclesList, step_size):
    
    x_start = start[0]
    y_start = start[1]
    x_goal = goal[0]
    y_goal = goal[1]
    n_obstacles = len(obstaclesList)
    threshold = step_size
    
    path = [[x_start,y_start]]
    
    
    while (cal_distance(x_start,y_start,x_goal,y_goal)) >= step_size :
        obj_vec = np.array([(x_goal-x_start),(y_goal-y_start)])/(np.sqrt((x_goal-x_start)**2 + (y_goal-y_start)**2))
        min_distances = np.zeros(n_obstacles)
        
        for i in range(n_obstacles):
            out = np.array(computeDistancePointToPolygon(obstaclesList[i],x_start,y_start))
            min_distances[i] = min(out[:,0])
            
        if all(x >= threshold for x in min_distances):
            x_start += step_size*obj_vec[0]
            y_start += step_size*obj_vec[1]
            path.append([x_start,y_start])
        
        else:
            obstacle_no = np.argmin(min_distances)
            new_direction = computeTangentVectorToPolygon(obstaclesList[obstacle_no],x_start,y_start)
            x_start += step_size*new_direction[0]
            y_start += step_size*new_direction[1]
            path.append([x_start,y_start]) 
                   
    return np.array(path)


