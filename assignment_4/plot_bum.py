#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import time

bot_2=list()
bot_3=list()
bot_4=list()
bot_5=list()
bot_6=list()
bot_7=list()


def odom2(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global bot_2
    bot_2.append([x,y])


def odom3(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global bot_3
    bot_3.append([x,y])


def odom4(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global bot_4
    bot_4.append([x,y])


def odom5(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global bot_5
    bot_5.append([x,y])


def odom6(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global bot_6
    bot_6.append([x,y])


def odom7(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    global bot_7
    bot_7.append([x,y])

rospy.init_node('plot', anonymous=True)

rospy.Subscriber('bot_2/odom', Odometry,odom2)  
rospy.Subscriber('bot_3/odom', Odometry,odom3)  
rospy.Subscriber('bot_4/odom', Odometry,odom4)  
rospy.Subscriber('bot_5/odom', Odometry,odom5)  
rospy.Subscriber('bot_6/odom', Odometry,odom6)  
rospy.Subscriber('bot_7/odom', Odometry,odom7)  

time.sleep(3)
r = rospy.Rate(30)

while True:
    b_2=np.array(bot_2)
    b_3=np.array(bot_3)
    b_4=np.array(bot_4)
    b_5=np.array(bot_5)
    b_6=np.array(bot_6)
    b_7=np.array(bot_7)
    plt.clf()
    plt.grid()
    plt.scatter(np.arange(0,len(b_2))/30,b_2[:,0],label="Bot_2")
    plt.scatter(np.arange(0,len(b_3))/30,b_3[:,0],label="Bot_3")
    plt.scatter(np.arange(0,len(b_4))/30,b_4[:,0],label="Bot_4")
    plt.scatter(np.arange(0,len(b_5))/30,b_5[:,0],label="Bot_5")
    plt.scatter(np.arange(0,len(b_6))/30,b_6[:,0],label="Bot_6")
    plt.scatter(np.arange(0,len(b_7))/30,b_7[:,0],label="Bot_7")
    plt.xlabel("Time")
    plt.ylabel(" x-coordinate value")
    plt.title("K=0.5")
    plt.legend(loc=2)
    plt.savefig('plot_50.png')
    r.sleep()




    