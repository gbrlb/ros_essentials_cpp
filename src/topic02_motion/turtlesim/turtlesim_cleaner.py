#!/usr/bin/env python3

# python lib
import math
import numpy as np
import time
import rospy
# ros msg
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# ros services
from std_srvs.srv import Empty

def poseCallback(pose_message):
    global x, y, yaw, yaw_deg
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta
    yaw_deg = math.degrees(pose_message.theta)
    # print "pose callback"
    # print ('x = {}'.format(pose_message.x)) #new in python 3
    # print ('y = %f' %pose_message.y) #used in python 2
    # print ('yaw = {}'.format(pose_message.theta)) #new in python 3
    # print ('yaw_deg = {}'.format(yaw_deg)) #new in python 3

def move(velocity_publisher, speed, distance, is_forward):
    #declare a Twist message to send velocity commands
    velocity_message = Twist()
    #get current location 
    # global x, y # only read no need to make global 
    # save the initial locations
    x0 = x 
    y0 = y

    if (is_forward):
        rospy.loginfo("Moves Fwd from:[x={:.4f}, y={:.4f}]  Distance:[{:.2f}] at Speed:[{:.4f}]".format(x0, y0, distance, speed))
        velocity_message.linear.x = abs(speed)
    else:
        rospy.loginfo("Moves Bwd from:[x={:.4f}, y={:.4f}]  Distance:[{:.2f}] at Speed:[{:.4f}]".format(x0, y0, distance, speed))
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0

    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(10) # 10hz

    while True :
            velocity_publisher.publish(velocity_message)

            loop_rate.sleep()
            # Euclidean norm calculate the distance p2p
            # distance_moved_old = abs(math.sqrt(((x - x0) ** 2) + ((y - y0) ** 2)))
            distance_moved = abs(math.hypot(x - x0, y - y0))

            # rospy.loginfo("Moves Frwrd: {:.4f} Current position: {:.4f}}".format(distance_moved, x))
            # print('distance_moved: {:.4f}'.format(distance_moved))
            # print('currt position: {:.4f}'.format(x))
            # check if exceed the distance
            if  not (distance_moved < distance):
                rospy.loginfo("Reached!")
                break
    #finally, stop the robot when the distance is moved
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)
    
def rotate (velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):
    velocity_message = Twist()
    # transform degrees to radians
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

    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    t0 = rospy.Time.now().to_sec()
    i = 0

    while True :
        if i % 10 == 0 :
            rospy.loginfo("Rotates:[theta={:.2f}deg]".format(yaw_deg))

        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        if  (current_angle_degree > relative_angle_degree):
            break
        loop_rate.sleep()
        i += 1

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)
    rospy.loginfo("Reached:[Theta={:.2f}deg]".format(yaw_deg))

def go_to_goal(velocity_publisher, x_goal, y_goal):
    # global x
    # global y, yaw

    velocity_message = Twist()
    loop_rate = rospy.Rate(100) # we publish the velocity at 10 Hz (10 times a second)    
    i = 0
    # proportional controler
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
        # print(i)
        if i % 100 == 0:
            # print ('x = ', x, ', y = ',y, ', distance to goal: ', distance)
            rospy.loginfo("\nMoves from:[x={:.4f}, y={:.4f}] to Goal:[x={:.4f}, y={:.4f}] at Speed:[linear{:.4f},agular{:.4f}] distance:[{:.4f}]".format(x, y, x_goal, y_goal, linear_speed, angular_speed, distance))
        if (distance < 0.01):
            break
        loop_rate.sleep()
        i += 1
    rospy.loginfo("Reached:[x={:.4f}, y={:.4f}]".format(x, y))
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)

def setDesiredOrientation(publisher, speed_in_degree, desired_absolute_angle_degree):
    relative_angle_radians = math.radians(desired_absolute_angle_degree) - yaw
    clockwise = 0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print ("relative_angle_radians: ",math.degrees(relative_angle_radians))
    print ("desired_absolute_angle_degree: ",desired_absolute_angle_degree)
    rotate(publisher, speed_in_degree,math.degrees(abs(relative_angle_radians)), clockwise)

def gridClean(publisher):
 
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0
 
    go_to_goal(publisher, desired_pose.x, desired_pose.y)
 
    setDesiredOrientation(publisher, 30, math.radians(desired_pose.theta))
 
    for i in range(5):
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, False)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, False)
    pass
 
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

if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous = True)

        # declare velocity publisher
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
        # define the subscriber (topic_name, msg_type, callback_function)
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(1)

        # move(velocity_publisher, speed=1.0, distance=4.0, is_forward=True)
        # rotate (velocity_publisher, angular_speed_degree=10, relative_angle_degree=90, clockwise=False)
        # go_to_goal(velocity_publisher, x_goal=9, y_goal=9)
        # setDesiredOrientation(publisher, speed_in_degree=30, desired_absolute_angle_degree=90)
        #setDesiredOrientation(velocity_publisher, 30, 90)

        opt = int(input("[1]-spiral cleaning | [2]-grid cleaning :"))
        print(opt)
        if opt == 1:
            spiralClean(velocity_publisher, 4, 0)
        elif opt == 2:
            gridClean(velocity_publisher)

    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("node terminated.")
