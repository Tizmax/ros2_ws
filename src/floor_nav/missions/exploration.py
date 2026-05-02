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
from geometry_msgs.msg import PoseStamped, Point, Pose, Pose2D

from kobuki_ros_interfaces.msg import Sound
from sensor_msgs.msg import BatteryState

rclpy.init(args=sys.argv)
tc = TaskClient('/floor_tasks', 0.2)

# --------------------- Global Variables ---------------------
going_base = False
battery_percentage = 100.0

# --------------------- Constants ---------------------
MIN_BATTERY_LEVEL = 50.0
MAX_EXPLORATION_TIME = 180.0
NOTHING_LEFT_TO_EXPLORE = False

# --------------------- Utils ---------------------
def error_callback(msg):
    global going_base
    if going_base:
        if msg.x < 0.1 and msg.y < 0.1 and msg.theta < pi/18:
            going_base = False

def battery_callback(msg):
    global battery_percentage
    battery_percentage = msg.percentage
    # tc.get_logger().info(f"Battery level: {battery_percentage}")

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

# ---------------------  ---------------------
tc.tf_buffer = tf2_ros.Buffer()
tc.tf_listener = tf2_ros.TransformListener(tc.tf_buffer, tc)
tc.enable_explorer_client = tc.create_client(SetBool, '/occgrid_planner/enable_explorer')

tc.batterySub = tc.create_subscription(BatteryState, '/sensors/battery_state', battery_callback, 1)
tc.soundPub = tc.create_publisher(Sound, '/commands/sound', 1)

# --------------------- UNDOCKING ---------------------
constant = tc.Constant(linear=-0.2,angular=0.0,duration=7.5)
constant = tc.Constant(linear=0.1,angular=0.0,duration=6)
# Tour complet pour générer la map
# tc.SetHeading(target=pi,relative=True,max_angular_velocity=0.3,angle_threshold=0.1,k_theta=3.0)
# tc.SetHeading(target=pi,relative=True,max_angular_velocity=0.3,angle_threshold=0.1,k_theta=3.0)

# On enregistre la pose face au dock
start_tf = tc.tf_buffer.lookup_transform( 
    "map", "base_link", rclpy.time.Time()
) 
# Demi tour
tc.SetHeading(target=pi,relative=True,max_angular_velocity=0.5,angle_threshold=0.1,k_theta=3.0)

tc.get_logger().info("Undocking completed")


# --------------------- EXPLORATION ---------------------

set_explorer_enabled(True)

tc.soundPub.publish(Sound(value=5))

start_time = rclpy.clock.Clock().now()

while True:

    # Check if we have reached the maximum exploration time
    elapsed_time = (rclpy.clock.Clock().now() - start_time).nanoseconds / 1e9
    if elapsed_time > MAX_EXPLORATION_TIME:
        tc.get_logger().info("Maximum exploration time reached, stopping exploration")
        break

    # Check if battery level is below threshold
    if battery_percentage < MIN_BATTERY_LEVEL:
        tc.get_logger().info("Battery level low, stopping exploration")
        break

    # Check if there is nothing left to explore
    if NOTHING_LEFT_TO_EXPLORE:
        tc.get_logger().info("Nothing left to explore, stopping exploration")
        break

    # wait a bit before checking again
    tc.Wait(duration=1.0)

# # stop the exploration task   
set_explorer_enabled(False)

tc.soundPub.publish(Sound(value=6))

# # --------------------- BACK TO BASE ---------------------


position = start_tf.transform.translation
tc.get_logger().info(f"Plan to {position}")

rotation = start_tf.transform.rotation
q = [rotation.x, rotation.y, rotation.z, rotation.w]
_, _, start_yaw = euler_from_quaternion(q)
tc.get_logger().info(f"Start yaw: {start_yaw:.3f} rad")

try:
    tc.PlanToEssential(goal_x=position.x, goal_y=position.y, goal_theta=start_yaw, dist_threshold=0.1, task_timeout=60.0)
except TaskException as e:
    tc.get_logger().info(f"Failed to plan back to base: {e}")

tc.GoToPose(goal_x=position.x,goal_y=position.y,goal_theta=start_yaw,max_velocity=0.3,k_v=0.3,max_angular_velocity=0.3, smart_control=False)


# --------------------- DOCKING ---------------------
for i in range(3):
    tc.get_logger().info(f"try {i+1} to dock...")
    try:
        tc.AutoDock(task_timeout=30.0)
        tc.get_logger().info(f"try {i+1} succeeded")
    except TaskException as e: 
        tc.get_logger().info(f"try {i+1} failed: {e}")
        constant = tc.Constant(linear=-0.1,angular=0.0,duration=7.5)


tc.get_logger().info("Docking completed")

tc.soundPub.publish(Sound(value=1))

# --------------------- MISSION COMPLETE ---------------------
tc.get_logger().info("Mission completed")
