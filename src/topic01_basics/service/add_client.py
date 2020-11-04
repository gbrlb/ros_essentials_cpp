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