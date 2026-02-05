#!/usr/bin/env python3
# ROS specific imports
import sys
import rclpy
from math import pi
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

scale=2.0
vel=0.5

try:
    tc.GoToPose(goal_x=-scale,goal_y=-scale,goal_theta=pi,max_velocity=vel,k_v=0.3,max_angular_velocity=0.3, smart_control= True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=-scale,goal_y=scale,goal_theta=3*pi/2,max_velocity=vel,k_v=0.3,max_angular_velocity=0.3, smart_control= True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=scale,goal_y=scale,goal_theta=0.0 ,max_velocity=vel,k_v=0.3,max_angular_velocity=0.3, smart_control= True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=scale,goal_y=-scale,goal_theta=pi/2,max_velocity=vel,k_v=0.3,max_angular_velocity=0.3, smart_control= True)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=-scale,goal_y=-scale,goal_theta=pi,max_velocity=vel,k_v=0.3,max_angular_velocity=0.3, smart_control= True)

except TaskException as e:
    tc.get_logger().error("Exception caught: " + str(e))


tc.get_logger().info("Mission completed")
