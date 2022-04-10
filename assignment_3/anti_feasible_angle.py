import numpy as np
import math

def nice_angle(angle):
    if angle<0:
        return angle+6.28319
    else:
        return angle
    
def col_cone(low_tangent, up_tangent):
    
    phi_1 = nice_angle(np.arctan2(low_tangent[1],low_tangent[0]))           # Looks
    phi_2 = nice_angle(np.arctan2(up_tangent[1],up_tangent[0]))
    
    phi_1 = np.round(phi_1*180/np.pi)
    phi_2 = np.round(phi_2*180/np.pi)
    
    if phi_1 > phi_2:
        temp_1 = set(np.arange(phi_2,phi_1,1))
        collision_cone = set(np.arange(0,360,1))-temp_1
        return np.array(list(collision_cone))
    else:
        collision_cone = np.arange(phi_1, phi_2,1)
        return collision_cone

    
