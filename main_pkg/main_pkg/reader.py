import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node


class MinimalGetParamClientAsync(Node):

    def __init__(self):
        super().__init__("param_reader_node")

        self.client = self.create_client(GetParameters, "/main/get_parameters")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.req = GetParameters.Request()

        param_to_read = ["target_obj"]
        response = self.send_request(param_to_read)

        print("Receive Param: %s" % (response.values[0].string_value))

    def send_request(self, params_name_list):
        self.req.names = params_name_list

        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_get_param_client = MinimalGetParamClientAsync()

    rclpy.shutdown()


if __name__ == "__main__":
    main()