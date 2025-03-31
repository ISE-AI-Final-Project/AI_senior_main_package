import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node


class ParamWriterNode(Node):
    def __init__(self):
        super().__init__("main")

        # Declare a parameter (optional if you just want to set it)
        param_descriptor = ParameterDescriptor(
            name="target_obj",
            type=ParameterType.PARAMETER_STRING,
            description="A sample parameter",
        )
        self.declare_parameter(
            "target_obj", "default_value", descriptor=param_descriptor
        )

        # Set a new value for the parameter
        new_value = "mumu"
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "target_obj", rclpy.Parameter.Type.STRING, new_value
                )
            ]
        )
        self.get_logger().info(f"'target_obj' set to: {new_value}")


def main(args=None):
    rclpy.init(args=args)
    node = ParamWriterNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()