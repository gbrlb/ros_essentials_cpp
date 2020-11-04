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
