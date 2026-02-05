#!/usr/bin/env python3
import sys
import rclpy
from math import pi
import random
from task_manager_client_py.TaskClient import *

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

while True:
    # Start WaitForFace as a background task
    face_detector = tc.WaitForFace(foreground=False)
    
    # Add condition to interrupt wandering when face is detected
    tc.addCondition(ConditionIsCompleted("face detector", tc, face_detector))
    
    try:
        # Wander until face detected (condition will interrupt this)
        tc.Wander(max_linear_speed=0.3, safety_range=0.8, dont_care_range=2.5,
                  max_angular_speed=0.3, multiplier=0.3, front_sector=True,
                  angular_range=pi/6, task_timeout=120.0)
        # Clear conditions if wander completes normally (shouldn't happen)
        tc.clearConditions()
    except TaskConditionException as e:
        # Face detected - interrupt from WaitForFace condition
        tc.get_logger().info("Face detected, stopping wander")
        # Conditions are cleared on trigger
        pass
    except TaskException as e:
        tc.get_logger().error("Error during wandering: %s" % str(e))
        pass
    
    # Stare at face - center it in image
    try:
        tc.StareAtFace(max_angular_velocity=0.3, face_center_threshold=0.2,
                      k_theta=0.8, center_face=True, image_width=640.0,
                      task_timeout=15.0)
    except TaskException as e:
        tc.get_logger().error("Error during staring at face: %s" % str(e))
        pass
    
    # Wait a few seconds while staring
    try:
        tc.Wait(duration=2.0)
    except TaskException as e:
        tc.get_logger().error("Error during wait: %s" % str(e))
        pass
    
    # Turn away (90 degrees or random)
    try:
        turn_angle = pi/2 + (random.random() - 0.5) * pi/3
        tc.SetHeading(target=turn_angle, relative=True,
                     max_angular_velocity=0.5, k_theta=1.0,
                     angle_threshold=0.05, task_timeout=10.0)
    except TaskException as e:
        tc.get_logger().error("Error during turn away: %s" % str(e))
        pass

tc.get_logger().info("Mission completed")