from tracemalloc import start
import numpy as np


x_start = None
y_start = None
x_goal = None
y_goal = None
step_size = None

lines = []
with open('input.txt', 'r') as file:
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
        # print("empty")
    else:
        lin = line.split(',')
        lin = np.float32(lin)
        obstacle.append([lin[0],lin[1]])
obstaclesList.append(obstacle)

