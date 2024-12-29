#!/usr/bin/env python3

from __future__ import division
import rospy
from geometry_msgs.msg import Twist, TransformStamped

from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
from math import atan, atan2, pi, sin, cos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import matplotlib
matplotlib.use('Agg')  # Use 'Agg' backend for rendering to files
import matplotlib.pyplot as plt

from course_project.msg import Trilateration
from trilateration import varA, varB, varC, varAbearing, varBbearing, varCbearing, landmarkA, landmarkB, landmarkC
import math
# Define global variables Here (pose, prev_pose, sys_input, sampling gain
# predicted_pose, estimated_pose)
pose = np.zeros((3, 1))
prev_pose = np.zeros((3, 1))
input_sys = np.zeros((2, 1))
our_predicted_pose = np.zeros((3, 1))
estimated_pose = np.zeros((3, 1))
odom_pose = np.zeros((3, 1))

# Define system noise related variables here
R = np.diag([0.01, 0.01, 1])  # Process Noise
Q = np.diag([varA, varB, varC, varAbearing, varBbearing, varCbearing])  # Measurement noise imported from trilateration
P = np.diag([1.0, 1.0, 1.0])  # Initial State Covariance
F = np.eye(3)  # System matrix for discretized unicycle model
I = np.eye(3)  # Identity matrix

FILTER_ORDER = 5  # Change filter settings
i = 0
filter_a = [0 for i in range(FILTER_ORDER)]
filter_b = [0 for i in range(FILTER_ORDER)]
filter_c = [0 for i in range(FILTER_ORDER)]

varX = 0.1
varY = 0.1
varTHETA = 0.05

idxA = 0
idxB = 0
idxC = 0
theta = 0

odoo = Odometry()
depub = ''

estimated_angle = 0.0 
predicted_angle = 0.0

prev_theta = 0
prev_x = 0
prev_y = 0


# Lists to store path data for plotting
actual_path = []
desired_path = []
odom_path = []
distance_error_l = []
angle_error_l = []
estimated_angle_l = []
odom_angle_l = []
vicon_path = []

def heading_from_quaternion(x, y, z, w):
    ang_1 = 2*(w*z + x*y)
    ang_2 = 1-2*(y**2 + z**2)
    return atan2(ang_1,ang_2) % (2*pi)

def get_current_H(pose, lA, lB, lC):
    # Calculate the linearized measurement matrix H(k+1|k) at the current robot pose

    # x and y co-ordinates of landmarks    
    # current robot pose
    # Write the code for calculating Linearized H here    
    
    x, y, theta = pose.flatten()
    H = []
    for lx, ly in [lA, lB, lC]:
        dx = lx - x
        dy = ly - y
        dist = sqrt(dx ** 2 + dy ** 2)

        H_range = [-dx / dist, -dy / dist, 0]

        H_bearing = [dy / (dist ** 2), -dx / (dist ** 2), -1]

        H.append(H_range)
        H.append(H_bearing)
    
    return np.array(H)

    # return 2 * np.array(H)


def sq_dist(p1, p2):
    # Given a pair of points the function returns euclidean distance
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)#**(0.5)


def predict_state(estimated_pose):
    # System evolution
    # global noisy_pose, pose, input_sys, F, P, our_predicted_pose, estimated_angle, predicted_angle 
    # input_sys.shape = (2, 1)
    
    global input_sys, F, Q, P
    
    theta = estimated_pose[2, 0]
    linear_vel = input_sys[0, 0]
    angular_vel = input_sys[1, 0]
    
    # Predict the new state based on the unicycle model
    predicted_pose = estimated_pose.copy()
    predicted_pose[0, 0] += linear_vel * cos(theta) * 0.2  # Assuming dt = 0.2
    predicted_pose[1, 0] += linear_vel * sin(theta) * 0.2
    predicted_pose[2, 0] += angular_vel * 0.2
    predicted_pose[2, 0] = atan2(sin(predicted_pose[2, 0]), cos(predicted_pose[2, 0]))
    
    # Update covariance prediction
    P = F @ P @ F.T + R
    
    return predicted_pose   
   
    # return predicted_pose 


def predict_measurement(predicted_pose, landmark_A, landmark_B, landmark_C):
    # Predicts the measurements (range and bearing) given the current position of the robot
    x, y, theta = predicted_pose.flatten()
    measurements = []

    for lx, ly in [landmark_A, landmark_B, landmark_C]:
        dx = lx - x
        dy = ly - y
        dist = sqrt(dx ** 2 + dy ** 2)

        # Range measurement
        range_meas = dist
        measurements.append(range_meas)

        # Bearing measurement
        bearing_meas = atan2(dy, dx) - theta
        bearing_meas = atan2(sin(bearing_meas), cos(bearing_meas))  # Normalize to [-pi, pi]
        measurements.append(bearing_meas)

    return np.array(measurements).reshape(6, 1)


def callback2(data):
    global noisy_pose, varX, varY, varTHETA, odom_pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    theta = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, z, w])[2]
    
    noise = [np.random.normal(0, varX), np.random.normal(0, varY), np.random.normal(0, varTHETA)]
    noisy_pose = np.array([data.pose.pose.position.x + noise[0], 
                           data.pose.pose.position.y + noise[1], 
                           euler_from_quaternion([x, y, z, w])[2] + noise[2]]).reshape(3, 1)
    
    x_pos = data.pose.pose.position.x
    y_pos = data.pose.pose.position.y
    odom_pose = np.array([x_pos, y_pos, theta]).reshape(3, 1)
    odom_path.append((x_pos, y_pos))
    
    theta = atan2(sin(theta), cos(theta))


    odom_angle_l.append(theta)
    
    pubop.publish(f"Odom Pose -> x: {x_pos}, y: {y_pos}, theta: {theta}")

    

# def callback_vicon(data):
#     global noisy_pose
#     pos_x = data.transform.translation.x
#     pos_y = data.transform.translation.y
#     orientation_q = data.transform.rotation
#     heading = heading_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
#     odom_pose = np.array([pos_x,pos_y, heading]).reshape(3,1)
#     odom_path.append((pos_x, pos_y))

#     heading = atan2(sin(heading), cos(heading))

#     odom_angle_l.append(heading)
    
#     pubop.publish(f"Odom Pose -> x: {pos_x}, y: {pos_y}, theta: {heading}")
    
def get_waypoint(t):
    global K_samp
    
    # Trajectory equations: xr(t) = A * sin(a * t + delta), yr(t) = B * sin(b * t)
    A, B = 1, 1  
    a, b = 1.0, 1.0  
    delta = 0.4  
    
    m = A * cos(a * t + delta)
    n = B * sin(b * t)
 
    return [m, n]

def callback(data):
    global distanceLandmarkA, distanceLandmarkB, distanceLandmarkC
    global idxA, idxB, idxC
    global filter_a, filter_b, filter_c
    global prev_pose, theta, pose
    global P, Q, R, F
    global estimated_pose, noisy_pose, our_predicted_pose, estimated_angle, predicted_angle, pose_list, prev_theta, prev_x, prev_y 

    lA = data.landmarkA
    lB = data.landmarkB
    lC = data.landmarkC

    #######################################################
    # FILTERING VALUES
    #######################################################
    # Add value into r buffer at indices idxA, idxB, idxC
    filter_a[idxA] = lA.distance
    filter_b[idxB] = lB.distance
    filter_c[idxC] = lC.distance
    
    # Increment indexes
    idxA += 1
    idxB += 1
    idxC += 1

    # wrap around the indices if buffer full
    if idxA >= FILTER_ORDER:
        idxA = 0
    if idxB >= FILTER_ORDER:
        idxB = 0
    if idxC >= FILTER_ORDER:
        idxC = 0

    # Calculate filtered measurements (d1, d2, d3)
    d1 = sum(filter_a) / FILTER_ORDER
    d2 = sum(filter_b) / FILTER_ORDER
    d3 = sum(filter_c) / FILTER_ORDER
    Y_measured = np.array([d1, lA.bearing, d2, lB.bearing, d3, lC.bearing]).reshape(6, 1)

    # z vector

    # EXTENDED KALMAN FILTER CODE GOES BELOW

    # Prediction:
    # you may use the function 'predict_state()' defined above as a substitute for prediction
    # x(k+1|k)
    predicted_pose = predict_state(estimated_pose)
    predicted_measurement = predict_measurement(predicted_pose, landmarkA, landmarkB, landmarkC)

    # Linearized Measurement Matrix
    H = get_current_H(predicted_pose, landmarkA, landmarkB, landmarkC)
    
    # Measurement Residual
    residual = Y_measured - predicted_measurement

    # Kalman Gain Calculation
    S = H @ P @ H.T + Q
    K = P @ H.T @ np.linalg.inv(S)

    # State Update
    estimated_pose = predicted_pose + K @ residual

    # Covariance Update
    P = (I - K @ H) @ P

    # Update estimated angle
    x = estimated_pose[0, 0]
    y = estimated_pose[1, 0]
    theta = estimated_pose[2, 0]
    
    theta = atan2(sin(theta), cos(theta))
    
    if abs(theta - prev_theta) > 0.2 and abs(theta - prev_theta) < 4.2:
        theta = prev_theta
        x = prev_x
        y = prev_y
        estimated_pose[2, 0] = theta
        estimated_pose[0, 0] = x
        estimated_pose[1, 0] = y
        
    prev_theta = theta
    prev_x = x
    prev_y = y

    odoo.pose.pose.position.x = x
    odoo.pose.pose.position.y = y
    quaternion_val = quaternion_from_euler(0, 0, theta)
    odoo.pose.pose.orientation.x = quaternion_val[0]
    odoo.pose.pose.orientation.y = quaternion_val[1]
    odoo.pose.pose.orientation.z = quaternion_val[2]
    odoo.pose.pose.orientation.w = quaternion_val[3]
    depub.publish(odoo)

    # Record the actual path of the robot
    estimated_angle_l.append(theta)
    actual_path.append((x, y))
    

def control_loop():
    global pose, depub, input_sys, pubw, pubep, pubop, estimated_pose, noisy_pose, our_predicted_pose, estimated_angle

    rospy.init_node('controller_node')
    pub_tt = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/trilateration_data', Trilateration, callback)
    rospy.Subscriber('/odom', Odometry, callback2)
    # rospy.Subscriber('/vicon/tb3_1/tb3_1', TransformStamped, callback_vicon)
    depub = rospy.Publisher('/odom2', Odometry, queue_size=10)

    pubw = rospy.Publisher('/bot_0/waypoint', String, queue_size=10)
    pubep = rospy.Publisher('/bot_0/estimatedpose', String, queue_size=10)
    pubop = rospy.Publisher('/bot_0/odompose', String, queue_size=10)
    
    # Setting the rate for loop execution
    rate = rospy.Rate(5)

    # Twist values to move the robot
    timer = 0

    
    while not rospy.is_shutdown():
        
        # Generate waypoint based on current time
        waypoint = get_waypoint(timer)
        x_ref, y_ref = waypoint
        desired_path.append((x_ref, y_ref))
        waypoint = get_waypoint(rospy.get_time())
        pubw.publish(f"Waypoint -> x: {x_ref}, y: {y_ref}")


        # Extract current estimated position
        x_est = estimated_pose[0, 0]
        y_est = estimated_pose[1, 0]
        current_theta = estimated_pose[2, 0]

        current_theta = atan2(sin(current_theta), cos(current_theta))

        pubep.publish(f"Estimated Pose -> x: {x_est}, y: {y_est}, theta: {current_theta}")

        
        # PID Controller Gains
        Kp_linear = 0.15
        Kp_angular = 0.15

        # Calculate errors
        error_x = x_ref - x_est
        error_y = y_ref - y_est
        
        # Calculate distance to the current waypoint
        distance_error = (error_x**2 + error_y**2)**0.5
        distance_error_l.append(distance_error)
      
        desired_theta = pi - atan2(x_ref, y_ref)
    
        angle_error = desired_theta - current_theta

        # Normalize angle error to be within [-pi, pi]
        angle_error = atan2(sin(angle_error), cos(angle_error))
            
        angle_error_l.append(angle_error)

        # PID Controller for linear velocity
        linear_velocity = Kp_linear * distance_error 
        
        # print("Proportional: {:.2f}, Derivative: {:.2f}".format(Kp_linear*distance_error, Kd_linear*linear_derivative))

        # PID Controller for angular velocity
        angular_velocity = (Kp_angular * angle_error) 
        
        # print("Angle Error: {:.2f}, Angular Derivative: {:.2f}".format(angle_error, angular_derivative))

        velocity_msg = Twist()
        velocity_msg.linear.x = linear_velocity #if cos(angle_error) > 0 else -linear_velocity
        velocity_msg.angular.z = angular_velocity
        prev_theta = current_theta
        prev_x = x_est
        prev_y = y_est

        # Limit velocities for safety
        # velocity_msg.linear.x = max(min(velocity_msg.linear.x, 0.5), -0.5)  # Limit linear velocity
        # velocity_msg.angular.z = max(min(velocity_msg.angular.z, 1.0), -1.0)  # Limit angular velocity

        input_sys[0] = velocity_msg.linear.x
        input_sys[1] = velocity_msg.angular.z
        timer = timer + 0.004 * pi
        pub.publish(velocity_msg)
        pub_tt.publish(velocity_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        # Plot paths if interrupted
        actual_path_np = np.array(actual_path)
        desired_path_np = np.array(desired_path)
        odom_path_np = np.array(odom_path)
        
        
        squared_errors = []
        
        if len(actual_path)//2 == len(desired_path) or len(actual_path)//2 == len(desired_path) - 1 or (len(actual_path)//2) - 1 == len(desired_path):  
            
            print(len(actual_path), len(desired_path))  
            i = 0
            j = 0
            while i<len(desired_path):
                squared_errors.append((actual_path[j][0] - desired_path[i][0])**2 + (actual_path[j][1] - desired_path[i][1])**2)
                if j+2 == len(actual_path):
                    break
                    
                j += 2
                i += 1
                
            mse = sum(squared_errors) / len(squared_errors)  
            print(f"Mean Squared Error (MSE) for trajectory tracking: {mse:.4f}")

            with open('/ros1_catkin_ws/src/mse.txt', 'w') as mse_file:
                mse_file.write(f"Mean Squared Error (MSE): {mse:.4f}\n")
        else:
            print("Error: Actual path and desired path lengths do not match. Cannot compute MSE.")
            print(len(actual_path), len(desired_path))  
        
        plt.figure()
        plt.plot(actual_path_np[5:, 0], actual_path_np[5:, 1], label='Estimated Path (EKF)', color='b')
        plt.plot(odom_path_np[:, 0], odom_path_np[:, 1], label='Odom Path', color='g', linestyle='-.')
        plt.plot(desired_path_np[:, 0], desired_path_np[:, 1], label='Desired Path', color='r', linestyle='--')
        
        plt.xlabel('X position')
        plt.ylabel('Y position')
        plt.title('Robot Path Tracking: Odom vs Estimated vs Desired')
        plt.legend()
        plt.grid()
        plt.savefig('/ros1_catkin_ws/src/path.png')
        
        plt.figure()
        plt.plot(range(len(angle_error_l)), angle_error_l, label='Angle error', color='b')
        plt.plot(range(len(distance_error_l)), distance_error_l, label='Distance Error', color='g')
        
        plt.title('Error Analysis: Angle vs Distance')
        plt.legend()
        plt.grid()
        plt.savefig('/ros1_catkin_ws/src/error.png')
        
        plt.figure()
        plt.plot(range(len(estimated_angle_l)), estimated_angle_l, label='Estimated Angle', color='b')
        plt.plot(range(len(odom_angle_l)), odom_angle_l, label='Odom Angle', color='g')
        
        plt.title('Angle Comparison: Estimated vs Odom')
        plt.legend()
        plt.grid()
        plt.savefig('/ros1_catkin_ws/src/angle.png')
        pass
        