#!/usr/bin/env python

import sys
import rospy
from ak_planner.srv import *
from geometry_msgs.msg import Vector3

def getPathPlan(start_pose, goal_pose):
    rospy.wait_for_service('plan_path')
    try:
        plan_path = rospy.ServiceProxy('plan_path', PlanPath)
        resp1 = plan_path(start_pose, goal_pose)
        return resp1.trajectory
    except:
        print "Service call failed: %s" %e

if __name__ == "__main__":

    start_pose = Vector3(0.0, 2.0, 0.0)
    goal_pose = Vector3(7.0, 2.0, 0.0)
    print "Requesting \n %s \n %s" %(start_pose, goal_pose)
    
    trajectory = getPathPlan(start_pose, goal_pose)
    print "Length of trajectory: %s" %(len(trajectory))
