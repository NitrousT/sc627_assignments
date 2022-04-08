from cv2 import phase
import numpy as np
import math

def feasible_ang(obs_rad, b_pose, o_pose, obs_vel, max_b_vel, num_ang=360):
    # First we calculate cone angles
    cone_hangle = math.asin(obs_rad/(np.sqrt((o_pose[0]-b_pose[0])**2+(o_pose[1]-b_pose[1])**2)))
    
    phi_1 = math.atan2(o_pose[1]-b_pose[1],o_pose[0]-b_pose[0]) - cone_hangle           # First angle
    phi_2 = math.atan2(o_pose[1]-b_pose[1],o_pose[0]-b_pose[0]) + cone_hangle           # Second angle
    
    values = []
    for i in range(num_ang):
        values.append((obs_vel[1]+ max_b_vel*math.sin(i*math.pi/180))/(obs_vel[0]+max_b_vel*math.cos(i*np.pi/180)))
        
    values = np.array(values)
    
    
    lower_idx = np.argmin(np.absolute(values-np.tan(phi_1)))
    upper_idx = np.argmin(np.absolute(values-np.tan(phi_2)))
    
    # print(lower_idx)
    # print(upper_idx)
    
    all_angles = np.arange(0,num_ang,1)
    
    if lower_idx > upper_idx:
        temp_1 = all_angles[lower_idx:]
        temp_2 = all_angles[:upper_idx]
        collision_range = np.concatenate((temp_1,temp_2))
        return collision_range
        
    else:
        collision_range = all_angles[lower_idx:upper_idx]
        return collision_range
    
    
    
