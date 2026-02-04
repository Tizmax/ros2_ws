import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        # subscribe to joystick messages
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription 

        # publish to the robot
        self.publisher = self.create_publisher(Twist, 'vrep/twistCommand', 10)


        self.declare_parameter('linear_scaling_factor', 1.0)
        self.declare_parameter('angular_scaling_factor', 1.0)
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)

    def set_speed(self, value):
        self.set_parameters([
            rclpy.parameter.Parameter(
                'linear_scaling_factor',
                rclpy.Parameter.Type.DOUBLE,
                value
            )
        ])

    def listener_callback(self, msg):

        # Update the parameter
        if msg.buttons[4]: # X
            self.set_speed(0.5)
        elif msg.buttons[0]:  # A
            self.set_speed(2.0)
        elif msg.buttons[1]:
            self.set_speed(1.0) #  B
        

        # get all parameters
        linear_axis = self.get_parameter('linear_axis').get_parameter_value().integer_value
        angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        linear_scaling_factor = self.get_parameter('linear_scaling_factor').get_parameter_value().double_value
        angular_scaling_factor = self.get_parameter('angular_scaling_factor').get_parameter_value().double_value

        # create the command
        cmd = Twist()
        cmd.linear.x = msg.axes[linear_axis] * linear_scaling_factor
        cmd.angular.z = msg.axes[angular_axis] * angular_scaling_factor
        self.publisher.publish(cmd)

def main():
    try:
        with rclpy.init():
            node = JoystickControlNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()