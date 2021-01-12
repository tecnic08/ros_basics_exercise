#!/usr/bin/env python3


import rospy
from ros_basics_msgs.msg import SimplePoseStamped


def spin():
    # 1) call the corresponding service to check if the waypoint was reached
    # 2) subscribe to the robot_pose topic to get the robot's current pose
    # 3) call the corresponding service to get the current waypoint or waypoint list
    # 4) implement your PID/PD/P logic
    # 5) publish your computed velocities in the set_velocities topic
    # 6) if there are no waypoints left then set the velocities to 0 and wait for the next waypoint

    # if you are using C++ then leave this function call empty !
    pass


if __name__ == '__main__':
    rospy.init_node('thymio_control_pnode', anonymous=True)
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        spin()
