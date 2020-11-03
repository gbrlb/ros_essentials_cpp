#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose #task 1. import the Pose type from the module turtlesim => msg type: [turtlesim/Pose]

# listenr type
def poseCallback(pose_message):
    #task 4. display the x, y, and theta received from the message
    print("pose callback:")
    print('x = {}'.format(pose_message.x)) #new in python 3
    print('y = %f' %pose_message.y) #used in python 2
    print('yaw = {}'.format(pose_message.theta)) #new in python 3

def listener():
    #initialize node
    node_name = 'turtlesim_motion_pose'
    rospy.init_node(node_name, anonymous=True)
    
    #task 2. subscribe to the topic of the pose of the Turtlesim
    position_topic = "/turtle1/pose"
    pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 

    #task 3. spin
    rospy.spin()    

if __name__ == '__main__':
    try:
        listener()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")