#!/usr/bin/env python3
from __future__ import division
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from math import cos, sin, pi
import numpy as np

import matplotlib.pyplot as plt

plt.ion() 

K_samp = 0
def get_waypoint():
    return (0,0) 


TRACE_PATH=True#False

LIM=4
fig, ax = plt.subplots(1, 1)
#ax.set_autoscaley_on(True)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_xlim([-LIM/2, LIM/2])
ax.set_ylim([-LIM/2, LIM/2])

ax.grid()
ax.legend()
line_waypoints = [[3,0]]
line_ep= [[0,0]]
line_poses = []

tracking_points = []


line_ep, = ax.plot([], [], 'bx', label="waypoint", ms=10)
line_poses2, = ax.plot([],[],'r', lw=3 , alpha=0.9 )
line_poses, = ax.plot([],[],'ro', label="robot", ms=10.0, alpha=0.5)
line_waypoints, = ax.plot([], [], 'g^', label="waypoint", ms=8)
line_waypoints2, = ax.plot([], [], 'g', lw=3, alpha=0.9)

track, = ax.plot([],[],'b:', lw=2, alpha=0.65)
        
X_track = []
Y_track = []
X_ref = []
Y_ref = []


def pose_listener( data):
    global line_poses, line_poses2, line_poses2, X_track, Y_track, tracking_points
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    
    #Complete the code to track the pose
    ## save the tracks to plot later  
     # Append current position to track
    X_track.append(pose[0])
    Y_track.append(pose[1])
    tracking_points.append(pose)

    # Update the trajectory plot
    line_poses.set_data(X_track, Y_track)  # Red dots for individual positions
    line_poses2.set_data(X_track, Y_track)  # Red line for the trajectory  
    

def waypoint_listener( data):
    global line_waypoints
    # Write your code here
    
    parts = data.data.split(',')
    x_part = parts[0].split(':')[-1].strip()
    y_part = parts[1].split(':')[-1].strip()
        
    waypoints_x = float(x_part)
    waypoints_y = float(y_part)
    X_ref.append(waypoints_x)
    Y_ref.append(waypoints_y)

    line_waypoints2.set_data(X_ref, Y_ref)
    line_waypoints.set_data(waypoints_x, waypoints_y)

    return line_waypoints

def ep_listener( data):
    global line_ep
    #Write you code here
    
    parts = data.data.split(',')
    x_part = parts[0].split(':')[-1].strip()
    y_part = parts[1].split(':')[-1].strip()
        
    estimated_x = float(x_part)
    estimated_y = float(y_part)

    line_ep.set_data(estimated_x, estimated_y)
    
    return line_ep
            
def process():
    
    rospy.init_node('plotting_node', anonymous=True)
    rospy.Subscriber('/odom2', Odometry, pose_listener)
    rospy.Subscriber('/bot_0/waypoint', String, waypoint_listener)
    rospy.Subscriber('/bot_0/estimatedpose', String, ep_listener)
    
    rate = rospy.Rate(5) # 10hz
    
    while not rospy.is_shutdown():
        fig.canvas.draw()
        fig.canvas.flush_events()
        
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Process started ")
        process()
    except rospy.ROSInterruptException:
        plt.xlabel('X (in meters)')
        plt.ylabel('Y (in meters)')
        plt.title('Robot Trajectory')
        plt.legend()
        plt.savefig('/ros1_catkin_ws/src/trajectory.png')
        plt.show()

        pass
    

        
