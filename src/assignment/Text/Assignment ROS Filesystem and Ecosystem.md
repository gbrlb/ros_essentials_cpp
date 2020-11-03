
# Assignment: ROS Filesystem and Ecosystem

Note: please use {...} to format the instruction for better readability and make it easier for me to review.

To respond to the questions, open a terminal and start turtlesim simulator and test commands before putting your answer.

- What is the first command you must run in ROS? <p><code>roscore</code></p>

- What is the command to run the Turtlesim simulator node?   <p><code>rosrun turtlesim turtlesim_node</code></p>

- What is the command to find the list of all ROS nodes?   <p><code>rosnode list</code></p>

- What is the command to find the list of all ROS topics?   <p><code>rostopic list</code></p>

- What is the topic that tells about the position of the turtle?   <p><code>/turtle1/pose</code></p>

- What is the topic that sends commands to the turtle to make it move? <p><code>/turtle1/cmd_vel</code></p>

- What is the command that tells you the information about the velocity topic? <p><code>rostopic info /turtle1/cmd_vel</code></p>

- What is the node used to publish the velocity commands to the turtle?  <p><code>/turtle1/cmd_vel</code></p>

- What is the node used to subscribe to the velocity commands to the turtle?   <p><code>rostopic pub</code></p>

- What is the command that allows to see the type of message for velocity topic?   <p><code>rosmsg show</code></p>

- What is the content of the velocity message? Explain its content.   
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

- What is the content of the position message? Explain its content.   

```console
$ rosmsg show geometry_msgs/Pose
geometry_msgs/Point position
  float64 x
  float64 y
  float64 z
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
```

- Write is the command that allows to publish velocity command to the turtle with a linear velocity 1.0 and angular velocity 0.5.

<p><code>rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'</code></p>


## Questions for this assignment

- 1. What is the first command you must run in ROS? <p><code>roscore</code></p>

- 2. What is the command to run the Turtlesim simulator? <p><code>rosrun turtlesim turtlesim_node</code></p>

- 3. What is the command to find the list of all ROS nodes? <p><code>rosnode list</code></p>

- 4. What is the command to find the list of all ROS topics? <p><code>rostopic list</code></p>

- 5. What is the topic that tells about the position of the turtle? <p><code>/turtle1/pose</code></p>

- 6. What is the topic that sends command to the turtle to make it move? <p><code>/turtle1/cmd_vel</code></p>

- 7. What is the command that tells you information about the topic about velocity? <p><code>rostopic info /turtle1/cmd_vel</code></p>

- 8. What is the node used to publish velocity commands to the turtle? <p><code>/teleop_turtle</code></p>

- 9. What is the node used to subscribe to velocity commands to the turtle?     <p><code>/turtlesim</code></p>

- 10. What is the command that allows to see the type of message for velocity topic?    <p><code>rosnode info /turtlesim</code></p> <p><code>rosnode info /teleop_turtle</code></p>

- 11. What is the content of the velocity message? Explain its content. 
  
```console
$ rosmsg show geometry_msgs/Twist
geometry_msgs/Vector3 linear # linear velocity of axes
  float64 x # velocity in x
  float64 y # velocity in y
  float64 z # velocity in z
geometry_msgs/Vector3 angular # angular velocity around axes
  float64 x # Roll
  float64 y # Pitch
  float64 z # Yaw
```

- 12. What is the content of the position message? Explain its content
the position message contents the posion of the robot in two dimentional space.

```console
$ rosmsg show turtlesim/Pose
float32 x # position in axis x 0 in LL
float32 y # position in axis y 0 in LL
float32 theta # orientation in rad
float32 linear_velocity # linear velocity
float32 angular_velocity # angular velocity
```