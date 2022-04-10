#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import time

bot_1=list()

def bot_odom(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global bot_1
    bot_1.append([x,y])


rospy.init_node('plot', anonymous=True)

rospy.Subscriber('bot_1/odom', Odometry, bot_odom)  


time.sleep(3)
r = rospy.Rate(30)

while True:
    b_1=np.array(bot_1)
    plt.clf()
    plt.grid()
    plt.plot(np.arange(0,len(b_1))/30,b_1[:,0],label="Bot")
    plt.plot()
    plt.xlabel("Time")
    plt.ylabel("y-coordinate")
    plt.title('X vs Time')
    plt.legend(loc=2)
    plt.savefig('plot_ass_3_1.png')
    r.sleep()




    