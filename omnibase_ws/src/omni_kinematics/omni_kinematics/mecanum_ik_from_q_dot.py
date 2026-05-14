import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Float32MultiArray
import numpy as np


class MecanumIKNode(Node):
    def __init__(self):
        super().__init__('mecanum_ik_node')

        # Declare pose velocity and geometry parameters
        self.declare_parameter('x_dot', 0.0)
        self.declare_parameter('y_dot', 0.0)
        self.declare_parameter('phi_dot', 0.0)
        self.declare_parameter('phi', 0.0)  # Heading
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('x_offset', 0.2)
        self.declare_parameter('y_offset', 0.2)

        # Publisher for calculated wheel speeds
        self.publisher_ = self.create_publisher(Float32MultiArray, '/mecanum_ik_u', 10)

        # Parameter client to update serial_comm_node's wheel velocities
        self.cli_node = 'serial_comm_node'
        self.cli = self.create_client(SetParameters, f'/{self.cli_node}/set_parameters')

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Get params
        x_dot = self.get_parameter('x_dot').value
        y_dot = self.get_parameter('y_dot').value
        phi_dot = self.get_parameter('phi_dot').value
        phi = self.get_parameter('phi').value
        r = self.get_parameter('wheel_radius').value
        x_off = self.get_parameter('x_offset').value
        y_off = self.get_parameter('y_offset').value
        l = x_off + y_off

        # Compute wheel speeds
        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)

        u = np.zeros(4)
        u[0] = (x_dot * (cos_phi + sin_phi) - y_dot * (cos_phi - sin_phi) - phi_dot * l) / r
        u[1] = (x_dot * (cos_phi - sin_phi) + y_dot * (cos_phi + sin_phi) + phi_dot * l) / r
        u[2] = (x_dot * (cos_phi + sin_phi) - y_dot * (cos_phi - sin_phi) + phi_dot * l) / r
        u[3] = (x_dot * (cos_phi - sin_phi) + y_dot * (cos_phi + sin_phi) - phi_dot * l) / r

        # Publish u to topic
        msg = Float32MultiArray()
        msg.data = u.tolist()
        self.publisher_.publish(msg)

        # Update serial_comm_node's parameters
        if self.cli.wait_for_service(timeout_sec=0.1):
            param_names = ['u1_desired', 'u2_desired', 'u3_desired', 'u4_desired']
            param_msgs = [
                Parameter(name=param_names[i], value=u[i]).to_parameter_msg() for i in range(4)
            ]
            from rcl_interfaces.srv import SetParameters
            request = SetParameters.Request()
            request.parameters = param_msgs
            self.cli.call_async(request)
        else:
            self.get_logger().warn(f"⚠️ Can't contact {self.cli_node}/set_parameters service.")


def main(args=None):
    rclpy.init(args=args)
    node = MecanumIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
