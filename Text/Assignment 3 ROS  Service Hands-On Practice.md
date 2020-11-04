# Assignment 3: Do-It-Yourself: ROS Service Hands-On Practice

## Follow the instructions below and respond to the questions:

1. Open the Turtlesim simulator
2. Display the list of services
3. What is the command the information of the service /reset
4. Write the result of the execution of the command for the service /reset
5. What is the command the information of the service /kill
6. Write the result of the execution of the command for the service /kill
7. spaw one additional turtle called tsim1. Write the command. 
8. spaw one additional turtle called tsim2. Write the command. 
9. use the service kill to kill tsim1.Write the command.  
10. use the service reset to reset all the simulation. Write the command. 

## Questions for this assignment

1. Open the Turtlesim simulator 
   `$ rosrun turtlesim turtlesim_node`
2. Display the list of services 
   ```
    $ rosservice list
    /clear
    /kill
    /reset
    /rosout/get_loggers
    /rosout/set_logger_level
    /spawn
    /turtle1/set_pen
    /turtle1/teleport_absolute
    /turtle1/teleport_relative
    /turtlesim/get_loggers
    /turtlesim/set_logger_level
    ```
3. What is the command that shows the information of the service /reset
    `rosservice info \reset`

4. Write the result of the execution of the command for the service /reset
    ```
    $ rosservice info \reset
    Node: /turtlesim
    URI: rosrpc://gbrl-ThinkPad-W520:51279
    Type: std_srvs/Empty
    Args
    ```
5. What is the command that shows the information of the service /kill
    `rosservice info \kill`

6. Write the result of the execution of the command for the service /kill
    ```
    $ rosservice info \kill
    Node: /turtlesim
    URI: rosrpc://gbrl-ThinkPad-W520:51279
    Type: turtlesim/Kill
    Args: name
    ```
7. What is the command that shows the content of message turtlesim/Kill of the /kill service?
    ```
    $ rossrv info turtlesim/Kill
    string name
    ---

    ```
8. Spaw one additional turtle called tsim1. Write the command.
    ```
    $ rosservice call /spawn 0 0 0 tsim1
    name: "tsim1"
    $ rosservice call /spawn 1 1 1 tsim2
    name: "tsim2"
    ```

10. use the service kill to kill tsim1.Write the command.
    `rosservice call /kill tsim1`

11. use the service reset to reset all the simulation. Write the command.
    `rosservice call /reset`
