import numpy as np
import math

def colcone_vec(bot_pose, obs_pose, obs_radius):
    
    dist2obs = np.sqrt((obs_pose[0]-bot_pose[0])**2+(obs_pose[1]-bot_pose[1])**2)
    cone_half_angle = np.arcsin(obs_radius/dist2obs)
    
    angle_2_center = math.atan2(obs_pose[1]-bot_pose[1], obs_pose[0]-bot_pose[0])
    
    if angle_2_center == 0.0:
        angle_2_center = 6.28319
    
    angle_tangent_low = angle_2_center - cone_half_angle
    angle_tangent_up = angle_2_center + cone_half_angle
    
    up_tangent = [dist2obs*math.cos(angle_tangent_up)*math.cos(angle_2_center+angle_tangent_up),dist2obs*math.cos(angle_tangent_up)*math.sin(angle_2_center+angle_tangent_up)]
    
    low_tangent = [dist2obs*math.cos(angle_tangent_low)*math.cos(angle_2_center+angle_tangent_low),dist2obs*math.cos(angle_tangent_low)*math.sin(angle_2_center+angle_tangent_low)]
    
    u_up_tangent = up_tangent/np.sqrt(up_tangent[0]**2+up_tangent[1]**2)
    u_low_tangent = low_tangent/np.sqrt(low_tangent[0]**2+low_tangent[1]**2)
    
    return u_up_tangent, u_low_tangent