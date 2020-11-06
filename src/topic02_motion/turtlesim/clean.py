#!/usr/bin/env python3
#ROS for Beginners Assigment 5 Gabriel Bermudez

import math
import numpy as np
import time
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty


def poseCallback(pose_message):
    global x, y, yaw, yaw_deg
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta
    yaw_deg = math.degrees(pose_message.theta)

def move(velocity_publisher, speed, distance, is_forward):
    velocity_message = Twist()
    x0 = x 
    y0 = y
    if (is_forward):
        rospy.loginfo("Moves Fwd from:[x={:.2f}, y={:.2f}]  Distance:[{:.2f}] at Speed:[{:.2f}]".format(x0, y0, distance, speed))
        velocity_message.linear.x = abs(speed)
    else:
        rospy.loginfo("Moves Bwd from:[x={:.2f}, y={:.2f}]  Distance:[{:.2f}] at Speed:[{:.2f}]".format(x0, y0, distance, speed))
        velocity_message.linear.x = -abs(speed)
    distance_moved = 0.0
    loop_rate = rospy.Rate(10) 
    while True :
            velocity_publisher.publish(velocity_message)
            loop_rate.sleep()
            distance_moved = abs(math.hypot(x - x0, y - y0))
            if  not (distance_moved < distance):
                break
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)
    rospy.loginfo("Move Reached:[x={:.2f}, y={:.2f}]".format(x, y))

def rotate (velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):
    velocity_message = Twist()
    angular_speed = math.radians(abs(angular_speed_degree))
    if (clockwise):
        abs_angle = yaw_deg - relative_angle_degree
        rospy.loginfo("Rotates CW from:[theta={:.2f}deg] to:[theta={:.2f}deg] at [angular_velocity:{:.2f}]".format(yaw_deg, abs_angle, angular_speed_degree))
        velocity_message.angular.z = -abs(angular_speed)
    else:
        abs_angle = yaw_deg + relative_angle_degree
        rospy.loginfo("Rotates CCW from:[theta={:.2f}deg] to:[theta={:.2f}deg] at [angular_velocity:{:.2f}]".format(yaw_deg, abs_angle, angular_speed_degree))
        velocity_message.angular.z = abs(angular_speed)
    angle_moved = 0.0
    loop_rate = rospy.Rate(50) 
    t0 = rospy.Time.now().to_sec()
    i = 0
    while True :
        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        if  (current_angle_degree > relative_angle_degree):
            break
        loop_rate.sleep()
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)
    rospy.loginfo("Rotation Reached:[Theta={:.2f}deg]".format(yaw_deg))

def go_to_goal(velocity_publisher, x_goal, y_goal):
    velocity_message = Twist()
    loop_rate = rospy.Rate(100) 
    rospy.loginfo("Moves from:[x={:.2f}, y={:.2f}] to Goal:[x={:.2f}, y={:.2f}]".format(x, y, x_goal, y_goal))
    i = 0
    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))
        linear_speed = distance * K_linear
        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        if (distance < 0.01):
            break
        loop_rate.sleep()
        i += 1
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)
    rospy.loginfo("Goal Reached:[x={:.2f}, y={:.2f}]".format(x, y))

def setDesiredOrientation(publisher, speed_in_degree, desired_absolute_angle_degree):
    relative_angle_radians = math.radians(desired_absolute_angle_degree) - yaw
    clockwise = 0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    rotate(publisher, speed_in_degree,math.degrees(abs(relative_angle_radians)), clockwise)

def gridClean(publisher):
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0
    go_to_goal(publisher, desired_pose.x, desired_pose.y)
    setDesiredOrientation(publisher, 30, math.radians(desired_pose.theta))
    for i in range(4):
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, False)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, False)
    rospy.loginfo("Grid Clean Done")

def spiralClean(velocity_publisher, wk, rk):
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
    while((x<10.5) and (y<10.5)):
        rk = rk+1
        vel_msg.linear.x = rk
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("Sipiral Clean Done")
    
if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous = True)
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(1)
        opt = int(input("[1]-spiral cleaning | [2]-grid cleaning :"))
        if opt == 1:
            spiralClean(velocity_publisher, wk=4, rk=1)
        elif opt == 2:
            gridClean(velocity_publisher)
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")