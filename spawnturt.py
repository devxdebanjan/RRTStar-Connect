#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn

def spawn_turtle(x, y, theta, name):
    rospy.wait_for_service('/spawn')    #spawning new turtle at desired position usinf rosservice
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, theta, name)
        rospy.loginfo("Spawned turtle %s at x=%f, y=%f, theta=%f", name, x, y, theta)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('spawn_turtle_node')
    spawn_turtle(5.0, 7.0, 0.0, 'my_turtle')
    rospy.spin()