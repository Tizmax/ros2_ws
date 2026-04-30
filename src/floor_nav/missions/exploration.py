#!/usr/bin/env python3
# ROS specific imports
import sys
import rclpy
from numpy import pi
import numpy.random
from task_manager_client_py.TaskClient import *
import tf2_ros
from tf_transformations import euler_from_quaternion
from std_srvs.srv import SetBool


rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

tc.tf_buffer = tf2_ros.Buffer()
tc.tf_listener = tf2_ros.TransformListener(tc.tf_buffer, tc)
tc.enable_explorer_client = tc.create_client(SetBool, '/occgrid_planner/enable_explorer')


def set_explorer_enabled(enable):
    request = SetBool.Request()
    request.data = enable

    while not tc.enable_explorer_client.wait_for_service(timeout_sec=1.0):
        tc.get_logger().info('Waiting for /occgrid_planner/enable_explorer service...')

    future = tc.enable_explorer_client.call_async(request)
    rclpy.spin_until_future_complete(tc, future)
    response = future.result()

    if response is None:
        raise RuntimeError('enable_explorer service call returned no response')

    if not response.success:
        raise RuntimeError(f'enable_explorer service failed: {response.message}')

    tc.get_logger().info(response.message)


# Constansts
MIN_BATTERY_LEVEL = 0.2
MAX_EXPLORATION_TIME = 30.0
NOTHING_LEFT_TO_EXPLORE = False


# --------------------- UNDOCKING ---------------------

constant = tc.Constant(linear=-0.3,angular=0.0,duration=5)
constant = tc.Constant(linear=0.2,angular=0.0,duration=3)
# Tour complet pour générer la map
# tc.SetHeading(target=pi,relative=True,max_angular_velocity=0.3,angle_threshold=0.1,k_theta=3.0)
# tc.SetHeading(target=pi,relative=True,max_angular_velocity=0.3,angle_threshold=0.1,k_theta=3.0)

# On enregistre la pose face au dock
start_tf = tc.tf_buffer.lookup_transform( 
    "map", "base_link", rclpy.time.Time()
) 
# Demi tour
tc.SetHeading(target=pi,relative=True,max_angular_velocity=0.3,angle_threshold=0.1,k_theta=3.0)

tc.get_logger().info("Undocking completed")


# --------------------- EXPLORATION ---------------------

set_explorer_enabled(True)

start_time = rclpy.clock.Clock().now()

# # Start EnableExplorer 
# tc.TaskEnableExplorer()

while True:

    # Check if we have reached the maximum exploration time
    elapsed_time = (rclpy.clock.Clock().now() - start_time).nanoseconds / 1e9
    if elapsed_time > MAX_EXPLORATION_TIME:
        tc.get_logger().info("Maximum exploration time reached, stopping exploration")
        break

    # Check if battery level is below threshold
    #   battery_level = 
    # if battery_level < MIN_BATTERY_LEVEL:
    #     tc.get_logger().info("Battery level low, stopping exploration")
    #     break

    # Check if there is nothing left to explore
    if NOTHING_LEFT_TO_EXPLORE:
        tc.get_logger().info("Nothing left to explore, stopping exploration")
        break

    # wait a bit before checking again
    tc.Wait(duration=1.0)

# # stop the exploration task   
# # tc.TaskDisableExplorer()
# tc.get_logger().info("Exploration completed")

# set_explorer_enabled(False)


# # --------------------- BACK TO BASE ---------------------

# constant = tc.Constant(linear=0.5,angular=0.0,duration=5)

base_x = start_tf.transform.translation.x
base_y = start_tf.transform.translation.y
q = [start_tf.transform.rotation.x, start_tf.transform.rotation.y, start_tf.transform.rotation.z, start_tf.transform.rotation.w]
_, _, base_theta = euler_from_quaternion(q)



tc.get_logger().info(f"Plan to ({base_x}, {base_y}, {base_theta})")

tc.PlanTo(goal_x=base_x, goal_y=base_y, goal_theta=base_theta)
tc.get_logger().info("Back to base completed")

# --------------------- DOCKING ---------------------

tc.AutoDock()

tc.get_logger().info("Docking completed")


# --------------------- MISSION COMPLETE ---------------------
tc.get_logger().info("Mission completed")
