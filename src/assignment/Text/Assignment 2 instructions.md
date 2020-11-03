# Assignment 2 instructions

This assignment will take you step by step to develop your first ROS program by applying the concept you have learned so far.

You can first watch the companion video before reading the following.

The objective is to develop a simple ROS program that:

- subscribes to the topic that gives information about the robot position.
- publishes to the topic that makes the robot move.
- develop a method called move(distance) which makes the robot moves a certain distance then stops.

- develop a method called rotate(distance) which makes the robot rotate a certain angle then stops.

We will use the Turtlesim simulator to develop this program.

## Start the Turtlesim Simulator

First, run the Turtlesim simulator using this command:

    > roscore
    > rosrun turtlesim turtlesim_node

You will have an interface similar to this one:

You will develop your first program to control this robot, STEP BY STEP following the instruction below.

We want to make this robot move and display its location. So let us get started.

## Questions for this assignment

1.  Find the topic name of the pose (position and orientation) of turtlesim and its message type. Display the content of message of the pose.
    ``` console
    $ rosnode info /turtlesim
    --------------------------------------------------------------------------------
    Node [/turtlesim]
    Publications:
    _ /rosout [rosgraph_msgs/Log]
    _ /turtle1/color_sensor [turtlesim/Color] \* /turtle1/pose [turtlesim/Pose]

        Subscriptions:
        * /turtle1/cmd_vel [geometry_msgs/Twist]

        Services:
        * /clear
        * /kill
        * /reset
        * /spawn
        * /turtle1/set_pen
        * /turtle1/teleport_absolute
        * /turtle1/teleport_relative
        * /turtlesim/get_loggers
        * /turtlesim/set_logger_level


        contacting node http://gbrl-ThinkPad-W520:40835/ ...
        Pid: 11963
        Connections:
        * topic: /rosout
            * to: /rosout
            * direction: outbound (35559 - 127.0.0.1:38826) [28]
            * transport: TCPROS
        * topic: /turtle1/cmd_vel
            * to: /rostopic_16620_1604369474477 (http://gbrl-ThinkPad-W520:43475/)
            * direction: inbound (46078 - gbrl-ThinkPad-W520:36509) [32]
            * transport: TCPROS
    ```

    > /turtle1/pose [turtlesim/Pose]

    ```
    $ rosmsg show turtlesim/Pose
    float32 x
    float32 y
    float32 theta
    float32 linear_velocity
    float32 angular_velocity

    ```

2.  Find the topic name of the velocity command of turtlesim and its message type. Display the content of message of the velocity command.
    Remember that velocity command is the topic that makes the robot move.

    > /turtle1/cmd_vel [geometry_msgs/Twist]

    ```
    $ rosmsg show geometry_msgs/Twist
    geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
    geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
    ```

3.  Write a simple ROS program called turtlesim_pose.py, which subscribes to the topic of the pose, and then prints the position of the robot in the callback function

    We provide you a program with missing code that you need to complete.

    ```
    #!/usr/bin/env python

    import rospy

    #task 1. import the Pose type from the module turtlesim

    def poseCallback(pose_message):

    #task 4. display the x, y, and theta received from the message
        print "pose callback"
        print ('x = '.%)
        print ('y = %f' %)
        print ('yaw = '.%)

    if __name__ == '__main__':
        try:

            rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #task 2. subscribe to the topic of the pose of the Turtlesim

        #task 3. spin


        except rospy.ROSInterruptException:
            rospy.loginfo("node terminated.")
    ```
    ```
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
    ```
4.  Complete the previous code to add a publisher to the velocity and make the robot move for a certain distance.

    Complete the missing code in this following program:

    ```
    #!/usr/bin/env python

    import rospy
    from geometry_msgs.msg import Twist
    from turtlesim.msg import Pose
    import math
    import time
    from std_srvs.srv import Empty

    x=0
    y=0
    z=0
    yaw=0

    def poseCallback(pose_message):
        global x
        global y, z, yaw
        x= pose_message.x
        y= pose_message.y
        yaw = pose_message.theta


    def move(speed, distance):
            #declare a Twist message to send velocity commands
            velocity_message = Twist()

                #get current location from the global variable before entering the loop
                x0=x
                y0=y
                #z0=z;
                #yaw0=yaw;

            #task 1. assign the x coordinate of linear velocity to the speed.


                distance_moved = 0.0
                loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)

                #task 2. create a publisher for the velocity message on the appropriate topic.
                velocity_publisher =

                while True :
                        rospy.loginfo("Turtlesim moves forwards")

                        #task 3. publish the velocity message


                        loop_rate.sleep()

                        #rospy.Duration(1.0)


                        #measure the distance moved
                        distance_moved = distance_moved+abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
                        print  distance_moved
                        if  not (distance_moved<distance):
                            rospy.loginfo("reached")
                            break

                #task 4. publish a velocity message zero to make the robot stop after the distance is reached



    if __name__ == '__main__':
        try:

            rospy.init_node('turtlesim_motion_pose', anonymous=True)

            #task 5. declare velocity publisher


            position_topic = "/turtle1/pose"
            pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)


            time.sleep(2)
            print 'move: '
            move (1.0, 5.0)
            time.sleep(2)
            print 'start reset: '
            rospy.wait_for_service('reset')
            reset_turtle = rospy.ServiceProxy('reset', Empty)
            reset_turtle()
            print 'end reset: '
            rospy.spin()

        except rospy.ROSInterruptException:
            rospy.loginfo("node terminated.")
    ```
    ```
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

    ```
