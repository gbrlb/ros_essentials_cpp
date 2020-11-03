#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist  # [geometry_msgs/Twist]
from turtlesim.msg import Pose  # [turtlesim/Pose]
import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
z = 0
yaw = 0


def poseCallback(pose_message):
    global x
    global y, z, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

    # print("pose callback")
    # print(('x = {}'.format(pose_message.x)) #new in python 3)
    # print(('y = %f' %pose_message.y) #used in python 2)
    # print(('yaw = {}'.format(pose_message.theta)) #new in python 3)


def move(speed, distance):
    # declare a Twist message to send velocity commands
    velocity_message = Twist()

    # get current location
    x0 = x
    y0 = y
    # z0=z;
    # yaw0=yaw;

    # task 1. assign the x coordinate of linear velocity to the speed.
    velocity_message.linear.x = speed
    distance_moved = 0.0

    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(10)

    # task 2. create a publisher for the velocity message on the appropriate topic.
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    while True:
        # rospy.loginfo("Turtlesim moves forwards")

        # task 3. publish the velocity message
        # print(velocity_message)
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

        # rospy.Duration(1.0)

        # measure the distance moved
        distance_moved = distance_moved + \
            abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print(distance_moved)
        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break

    # task 4. publish a velocity message zero to make the robot stop after the distance is reached
    # finally, stop the robot when the distance is moved
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def publisher_motion():
    node_name = 'turtlesim_motion_pose'
    rospy.init_node(node_name, anonymous=True)

    # task 5. declare velocity publisher
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    position_topic = "/turtle1/pose"
    pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
    time.sleep(2)

    print('move: ')
    #move(speed, distance)
    move(.10, 15)
    time.sleep(2)

    print('start reset: ')
    rospy.wait_for_service('reset')
    reset_turtle = rospy.ServiceProxy('reset', Empty)
    reset_turtle()

    print('end reset: ')
    rospy.spin()


if __name__ == '__main__':
    try:
        publisher_motion()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
