- [1. ROS for Beginners: Basics, Motion, and OpenCV](#1-ros-for-beginners-basics-motion-and-opencv)
  - [1.1. Introduction](#11-introduction)
  - [1.2. ROS: How did it revolutionize robotics software development](#12-ros-how-did-it-revolutionize-robotics-software-development)
  - [1.3. [NEW] Setting your environment with ROS Noetic](#13-new-setting-your-environment-with-ros-noetic)
  - [1.4. [LEGACY] Installation and Environment Setup](#14-legacy-installation-and-environment-setup)
  - [1.5. Create a ROS Workspace and a ROS Package](#15-create-a-ros-workspace-and-a-ros-package)
    - [1.5.1. Creating a ROS Workspace](#151-creating-a-ros-workspace)
    - [1.5.2. Creating a ROS Package](#152-creating-a-ros-package)
  - [1.6. [NEW] ROS Computation Graph](#16-new-ros-computation-graph)
  - [1.7. ROS Topics](#17-ros-topics)
  - [1.8. ROS Messages](#18-ros-messages)
    - [1.8.1. Steps to Create a Client?Server ROS Service App](#181-steps-to-create-a-clientserver-ros-service-app)
    - [1.8.2. Assignment 3: Do-It-Yourself: ROS Service Hands-On Practice](#182-assignment-3-do-it-yourself-ros-service-hands-on-practice)
  - [1.9. ROS Services](#19-ros-services)
    - [1.9.1. Creating Service File](#191-creating-service-file)
    - [Create the Server](#create-the-server)
    - [Create the Service Client Proxy](#create-the-service-client-proxy)
    - [1.9.2. Assignment 4: Rectangle Aera ROS Service (with video solution)](#192-assignment-4-rectangle-aera-ros-service-with-video-solution)
  - [1.10. [NEW] Motion in ROS (updated with ROS Noetic)](#110-new-motion-in-ros-updated-with-ros-noetic)
  - [1.11. Appendix: Motion in ROS (old videos - but still applicable)](#111-appendix-motion-in-ros-old-videos---but-still-applicable)
  - [1.12. ROS Tools and Utilities](#112-ros-tools-and-utilities)
  - [1.13. Getting Started with Turtlebot3](#113-getting-started-with-turtlebot3)
  - [1.14. Perception I: Computer Vision in ROS with OpenCV](#114-perception-i-computer-vision-in-ros-with-opencv)
  - [1.15. Perception II: Laser Range Finders (Laser Scanner)](#115-perception-ii-laser-range-finders-laser-scanner)
  - [1.16. rosserial: Connecting new Hardware (Arduino) with ROS](#116-rosserial-connecting-new-hardware-arduino-with-ros)
  - [1.17. Bonus](#117-bonus)
  - [1.18. Assigments](#118-assigments)
  - [1.19. Test](#119-test)
## 1. ROS for Beginners: Basics, Motion, and OpenCV

### 1.1. Introduction

### 1.2. ROS: How did it revolutionize robotics software development

### 1.3. [NEW] Setting your environment with ROS Noetic

### 1.4. [LEGACY] Installation and Environment Setup

### 1.5. Create a ROS Workspace and a ROS Package

`roscd`: takes you to the default ROS workspace

`catkin_make`:

`source /home/user/catkin_ws/delevel/setup.bash`

`package.xml` :

- is used to describe the package and set its de[endencies]
- is generated automatically when creating a new ros pakage.
- Defines two types of dependencies:
  - dependencies needed to build a package
  - dependencies needed to executte the package
- you can define a licencse of your package

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_package std_msgs rospy roscpp
```

`CmakeList.txt`:

- Describes how to build thw cpde and where to install it to
- Is the input to the Cmake build system fpr builden software packages

`roscd`

this command takes you to the last ROS workspace that you have sourced its setup.bash.

It is a good practice to source your overlay workspace in the .bashrc rather than sourcing it every time when you open a new terminal

#### 1.5.1. Creating a ROS Workspace
1. Source enviroment:
    ``` bash
    $ source /opt/ros/<vesion>/setup.bash
    # source /opt/ros/melodic/setup.bash
    ```
2. Create and build a catkin workspace:
    ```bash
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws
    $ catkin_make
    ```
3. Source new workspace, these will overlay this workspace on top your enviroment.
    ```bash
    $ cd ~/catkin_ws/
    $ source devel/setup.bash
    ```
4. Add your workspace to the ​.bashrc​ file such that it is sourced every time you start anew shell (terminal).
    ```bash
    $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    # optional add, make terminal a little slow to start
    $ echo "echo $ROS_PACKAGE_PATH" >> ~/.bashrc
    ```
5. To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in. check output
    ```bash
    $ echo $ROS_PACKAGE_PATH
    /home/<user_name>/catkin_ws/src:/opt/ros/<ROSDISTRO_NAME>/share
    ```
6. Test:
    ```bash
    $ roscd
    /catkin_ws/devel
    ```

#### 1.5.2. Creating a ROS Package
1. Go to a workspace/src
   ```bash
   $ cd ~/catkin_ws/src
   ```
   
2. Create new package, eg.
   ```shell
   # catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
   $ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
   ```
   This will create a beginner_tutorials folder which contains a package.xml and a CMakeLists.txt, which have been partially filled out with the information you gave catkin_create_pkg.

3. Compile new package, first go to root workspace directory:
    ```shell
    $ cd ~/catkin_ws
    $ catkin_make
    ```
   - src: folder to create new source programs
   - include: libraries included
4. Check package dependencies:
   ```shell
   # First-order dependencies
   $ rospack depends1 ros_essentials_cpp
   $ roscd ros_essentials_cpp
   $ cat package.xml

   # Indirect dependencies
   $ rospack depends1 rospy
   $ rospack depends ros_essentials_cpp

   
   ```

### 1.6. [NEW] ROS Computation Graph

Assignment 1: ROS Filesystem and Ecosystem

### 1.7. ROS Topics

Assignment 2: Do-It-Yourself: Write your First ROS Program to Control the Motion of a Robot

### 1.8. ROS Messages

#### 1.8.1. Steps to Create a Client?Server ROS Service App

1. Define the service message (service file)

2. Create ros Server node
   
3. Creat ROS Client node
   
4. Excecute the service
   
5. Consume the service by the client

#### 1.8.2. Assignment 3: Do-It-Yourself: ROS Service Hands-On Practice



### 1.9. ROS Services

**Steps to Create a Client/Server ROS Service App:**
1. Define the service message (service file)
   1. Message for send the **Request**
   2. Message for receive the **Response**
2. Create ROS Server node: will act as a server and will recive the inputs argumens from the client and will make the processing
3. Creat ROS Client node: 
4. Excecute the service   
  `rosrun ros_essentials_cpp add_server.py`
5. Consume the service by the client
  `rosrun ros_essentials_cpp add_client.py`
      
#### 1.9.1. Creating Service File
Create the servise file inside the directory: `/home/$USER/catkin_ws/src/<pkg_name>/srv`
1. Create the service file
      ```bash
      $ cd /home/$USER/catkin_ws/src/<pkg_name>/src
      $ > <file_name.srv>
      $ nano <file_name.srv>
      # Service Definition:
      # 1. REQUEST parameters to be sent from client to server
      int64 a
      int64 b
      # --- separation for the request and response parts
      ---
      # 2. RESPONSE parameter to be sent from server to the client
      int64 sum
      ```
2. Add these two dependencies in `package.xml`
   ```bash
   $ cd /home/$USER/catkin_ws/src/ros_service_assignment
   $ nano package.xml

   <build_depend>message_generation</build_depend>

   <exec_depend>message_runtime</exec_depend>
   ```
3. Check `CMakeLists.txt`:
   - add message_generation
   - add the service files
   - and uncomment generate_messages
    ```bash
    $ gedit CMakeLists.txt
    ## Find catkin macros and libraries
    find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation # <- create for message generation
    )

    ## Generate services in the 'srv' folder
    add_service_files(
    FILES
    AddTwoInts.srv # <- file created in ~/catkin_ws/src/pkg_dir/srv
    )


    ## Generate added messages and services with any dependencies listed here
    generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    )
    ``` 
4. build the new service and generate executable files: 
   ```
   $ cd catkin_ws/
   $ catkin_make
   ```
5. Make sure that service files are now created:
   - Check files `*.h` created in `~/catkin_ws/devel/include/<pkg_name>/`
   - `rossrv list` to show all servers
   - `rossrv show ros_service_assignment/RectangleAeraService`: show only spewcific service of the package
   - `rossrv show RectangleAeraService`: show all packages that use the service

#### Create the Server
```PYTHON
#!/usr/bin/env python

# import server request and response
# /home/$USER/catkin_ws/devel/include/ros_essentials_cpp
from ros_essentials_cpp.srv import AddTwoInts # service_type
from ros_essentials_cpp.srv import AddTwoIntsRequest # request
from ros_essentials_cpp.srv import AddTwoIntsResponse # response
#  python library
import rospy

# callback function(server_msg.srv) <- /home/$USER/catkin_ws/src/ros_essentials_cpp/srv
# req -> int64 a int64 b
def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)


def add_two_ints_server():
    # node_name = 'add_two_ints_server'
    rospy.init_node('add_two_ints_server')
    # rospy.Service('service_name', service_type, callback_function)
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin() # Stat the server, waiting for a request 


if __name__ == "__main__":
    add_two_ints_server()

```
#### Create the Service Client Proxy
```python
#!/usr/bin/env python

#  python library
import sys
import rospy
# /home/$USER/catkin_ws/devel/include/ros_essentials_cpp
from ros_essentials_cpp.srv import AddTwoInts # service_type
from ros_essentials_cpp.srv import AddTwoIntsRequest # request
from ros_essentials_cpp.srv import AddTwoIntsResponse # response


def add_two_ints_client(x, y):
    # wait until the server start
    rospy.wait_for_service('add_two_ints')
    try:
        # rospy.ServiceProxy('service_name', service_type)
        # make sure that 'service_name' is the same that the server
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        # call server method 'service_name'
        resp1 = add_two_ints(x, y)
        # resp1 -> int64 sum <- /home/$USER/catkin_ws/src/ros_essentials_cpp/srv
        return resp1.sum
    except rospy.ServiceException(e):
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    s = add_two_ints_client(x, y)
    print("%s + %s = %s"%(x, y, s))
```

#### 1.9.2. Assignment 4: Rectangle Aera ROS Service (with video solution)

In this exercise, you will develop a new ROS service by applying the concepts that you have learned in this section.
The objective is to create a ROS service where a client sends two float numbers, width and height to the server, then the server will respond the area of the rectangle.

First, create a new ROS package, and call it `ros_service_assignment`.

You first need to define a service file called `RectangleAeraService.srv`.

The **request** part should contains two float values: the **width** and the **height**. Use **float32** as a type. Refer to this ROS page for more information about available types in ROS. http://wiki.ros.org/msg

The **response** part should contain the **area** of the rectangle, which is **width*height**.

Write a Client and Server that implements this application using Python or C++.

Test your application and make sure that it works.

**Questions for this assignment**

1. What is the command used to create a ROS package called ros_service_assignment?
   Make sure to clarify the path where to create it.
    ```bash
    $ cd catkin_ws/
    $ cd src/
    $ catkin_create_pkg ros_service_assignment std_msg rospy roscpp
    Created file ros_service_assignment/package.xml
    Created file ros_service_assignment/CMakeLists.txt
    Created folder ros_service_assignment/include/ros_service_assignment
    Created folder ros_service_assignment/src
    Successfully created files in /home/$USER/catkin_ws/src/ros_service_assignment. Please adjust the values in package.xml.
    $ cd ..
    $ catkin_make
    ```

2. What is the name of the folder when to create the service file?
  Provide the absolute path to the file (from the root).
  `/home/$USER/catkin_ws/src/ros_service_assignment/srv`
3. What is the content of the service file RectangleAreaService.srv?
    ``` bash
    $ rossrv list
    $ rossrv show RectangleAeraService
    [ros_service_assignment/RectangleAeraService]:
    float32 width
    float32 height
    ---
    float32 area
    ```
4. What are the changes you need to do in the CMakeLists.txt. Copy/paste the whole CMakeLists.txt.
   
5. What are the changes you need to do the package.xml? Copy/paste the whole package.xml.
   
6. What is the command to build the new service and generate executable files?
   
7. How to make sure that service files are now created?
   
8. Write the **server** application (C++ or Python)
    ```python
    #!/usr/bin/env python

    # import server request and response
    # /home/$USER/catkin_ws/devel/include/ros_service_assignment
    from ros_service_assignment.srv import RectangleAeraService # service_type
    from ros_service_assignment.srv import RectangleAeraServiceRequest # request
    from ros_service_assignment.srv import RectangleAeraServiceResponse # response
    #  python library
    import rospy

    # callback function(server_msg.srv) <- /home/$USER/catkin_ws/src/ros_service_assignment/srv
    def handle_rectangle_area(req):
        Area = req.width * req.height
        print("Returning [%s * %s = %s]" % (req.width, req.height, (Area)))
        return RectangleAeraServiceResponse(Area)


    def rectangle_area_server():
        # node_name = 'rectangle_area_server'
        rospy.init_node('rectangle_area_server')
        # rospy.Service('service_name', service_type, callback_function)
        s = rospy.Service('rectangle_area', RectangleAeraService, handle_rectangle_area)
        print("Ready calculate the area:")
        rospy.spin() # Stat the server, waiting for a request 


    if __name__ == "__main__":
        rectangle_area_server()
    ```
9.  Write the **client** application (C++ or Python)
    ```python
    #!/usr/bin/env python

    #  python library
    import sys
    import rospy
    # /home/$USER/catkin_ws/devel/include/ros_service_assignment
    from ros_service_assignment.srv import RectangleAeraService # service_type
    from ros_service_assignment.srv import RectangleAeraServiceRequest # request
    from ros_service_assignment.srv import RectangleAeraServiceResponse # response


    def rectangle_area_client(x, y):
        # wait until the server start
        rospy.wait_for_service('rectangle_area')
        try:
            # rospy.ServiceProxy('service_name', service_type)
            # make sure that 'service_name' is the same that the server
            rectangle_area = rospy.ServiceProxy('rectangle_area', RectangleAeraService)
            # call server method 'service_name'
            resp1 = rectangle_area(x, y)
            # resp1 -> int64 sum <- /home/$USER/catkin_ws/src/ros_service_assignment/srv
            return resp1.area
        except:
            e = rospy.ServiceException
            print("Service call failed: %s" % e)

    def usage():
        return "%s [x y]"%sys.argv[0]

    if __name__ == "__main__":
        if len(sys.argv) == 3:
            # print('argv0:', sys.argv[0]) 
            #  argv[0] <- path
            x = int(sys.argv[1])
            y = int(sys.argv[2])
        else:
            print(usage())
            sys.exit(1)
        print("Requesting %s+%s"%(x, y))
        s = rectangle_area_client(x, y)
        print("Done: %s * %s = %s"%(x, y, s))
    ```
10. What are the commands to test that the application works fine.
     ```
     $ roscore
     $ rosrun ros_service_assignment RectangleAera_server.py
     $ rosrun ros_service_assignment RectangleAera_client.py 3 2
     ```
   
### 1.10. [NEW] Motion in ROS (updated with ROS Noetic)

### 1.11. Appendix: Motion in ROS (old videos - but still applicable)

### 1.12. ROS Tools and Utilities

### 1.13. Getting Started with Turtlebot3

### 1.14. Perception I: Computer Vision in ROS with OpenCV

### 1.15. Perception II: Laser Range Finders (Laser Scanner)

### 1.16. rosserial: Connecting new Hardware (Arduino) with ROS

### 1.17. Bonus

### 1.18. Assigments

- [ ] @mentions, #refs, [links](), **formatting**, and <del>tags</del> supported
- [ ] list syntax required (any unordered or ordered list supported)
- [ ] this is a complete item
- [ ] this is an incomplete item

### 1.19. Test
