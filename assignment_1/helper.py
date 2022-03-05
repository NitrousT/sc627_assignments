import dis
import numpy as np
import math


def cal_distance(x1,y1,x2,y2):
    return np.sqrt((x1-x2)**2 + (y1-y2)**2)

def cal_angle(x1,y1,x,y,x2,y2):
    seg_1 = [(x1-x),(y1-y)]
    seg_2 = [(x2-x),(y2-y)]
    mag_1 = np.sqrt((x1-x)**2 + (y1-y)**2)
    mag_2 = np.sqrt((x2-x)**2 + (y2-y)**2)
    
    return math.degrees(math.acos(np.dot(seg_1,seg_2)/(mag_1*mag_2)))


def computeLineThroughTwoPoints(x1,y1,x2,y2):
    a = (x2-x1)/(np.sqrt((x2-x1)**2 + (y1-y2)**2))
    b = (y1-y2)/(np.sqrt((x2-x1)**2 + (y1-y2)**2))
    c = ((y2-y1)*x1 + (x1-x2)*y1)/(np.sqrt((x2-x1)**2 + (y1-y2)**2))
    
    return a,b,c                        


def computeDistancePointToLine(x,y,x1,y1,x2,y2):
    a,b,c = computeLineThroughTwoPoints(x1,y1,x2,y2) 
    return a*y + b*x + c


def computeDistancePointToSegment(x,y,x1,y1,x2,y2):
    
    dist_1 = cal_distance(x1,y1,x,y)
    dist_2 = cal_distance(x2,y2,x,y)
    angle_2 = cal_angle(x,y,x1,y1,x2,y2)
    angle_3 = cal_angle(x,y,x2,y2,x1,y1)
    
    if angle_2 > 90.0 or angle_3 > 90.0:   
        if dist_1 < dist_2:
            return dist_1,1
        else:
            return dist_2,2
    else:
        return np.absolute(computeDistancePointToLine(x,y,x1,y1,x2,y2)),0       
    
    
def computeDistancePointToPolygon(points,x,y):
    n = len(points)
    min_dist = []
    for i in range(n):
        if i == n-1:
            i = -1
        min_dist.append(computeDistancePointToSegment(x,y,points[i][0],points[i][1],points[i+1][0], points[i+1][1]))   
   
    return (min_dist)


def computeTangentVectorToPolygon(points,x,y):
    
    min_dists = computeDistancePointToPolygon(points,x,y)
    min_dist = min(min_dists)
    min_dists = np.array(min_dists)
    line_no = np.argmin(min_dists[:,0])
    dist_type = min_dists[line_no,1]
    
    if line_no == len(points)-1:
        line_no = -1
        
    if min_dist[1] == 0.0:
        temp = np.array([(points[line_no+1][0]-points[line_no][0]),(points[line_no+1][1]-points[line_no][1])])
        mag = np.sqrt(temp[0]**2 + temp[1]**2)
        return temp/mag
    
    elif min_dist[1] == 1.0:
        temp = np.array([(x-points[line_no][0]),(y-points[line_no][1])])
        mag = np.sqrt(temp[0]**2 + temp[1]**2)
        vec = temp/mag
        temp2 = np.zeros(2)
        temp2[0] = -vec[1]
        temp2[1] = vec[0]
        return temp2
    else:
        temp = np.array([(x-points[line_no+1][0]),(y-points[line_no+1][1])])
        mag = np.sqrt(temp[0]**2 + temp[1]**2)
        vec = temp/mag
        temp2 = np.zeros(2)
        temp2[0] = -vec[1]
        temp2[1] = vec[0]
        return temp2
    


