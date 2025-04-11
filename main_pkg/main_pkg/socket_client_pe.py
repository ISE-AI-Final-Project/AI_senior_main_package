import cv2
import rclpy
from rclpy.node import Node
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import imageio
from geometry_msgs.msg import PoseStamped


from .utils.my_custom_socket import MyClient

def result_to_posestamped(res_list, node):
    if len(res_list) != 12:
        print('Wrong Dimension, Exiting the program')
        exit()
    
    rot_matrix = np.array(res_list[:9]).reshape((3, 3))
    translation = res_list[9:]

    quat = R.from_matrix(rot_matrix).as_quat()

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = 'zed2i_cam_left'
    pose_msg.header.stamp = node.get_clock().now().to_msg()

    pose_msg.pose.position.x = translation[0]
    pose_msg.pose.position.y = translation[1]
    pose_msg.pose.position.z = translation[2]

    pose_msg.pose.orientation.x = quat[0]
    pose_msg.pose.orientation.y = quat[1]
    pose_msg.pose.orientation.z = quat[2]
    pose_msg.pose.orientation.w = quat[3]

    return pose_msg


class SocketClientNode(Node):
    def __init__(self):
        super().__init__("socket_client_node")
        self.client = MyClient(host="127.0.0.1", port=65432)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.send_once = False

    def timer_callback(self):
        if self.send_once:
            return
        
        client_connected = self.client.connect()
        if not client_connected:
            self.get_logger().error(f"Server not started")
            return
        
        object_name = str(input("Please input Object's name : "))
        
        mask_path = '/home/bham/Desktop/test_scene/'+object_name+'.png'
        rgb_path = '/home/bham/Desktop/test_scene/rgb.png'
        depth_path = '/home/bham/Desktop/test_scene/depth.png'
        mask_image = Image.open(mask_path).convert("1")
        mask = np.array(mask_image, dtype=np.uint8)

        rgb_image = imageio.imread(rgb_path)      # (H, W, 3)
        depth_image = imageio.imread(depth_path)  # (H, W) -> uint16 or float32

        if mask is None:
            self.get_logger().error(f"[Client] Failed to read mask at {mask_path}")
            return

        try:
            result = self.client.request_msg(
                msg_type_in=["image2d", "image3d", "image2d", "string"],
                msg_in=[mask, rgb_image, depth_image, object_name])
            # self.get_logger().info(f"Response: {result}")
            posestamped_msg = result_to_posestamped(result, self)
            self.get_logger().info(f"\n{posestamped_msg}")

            self.send_once = True
            self.client.disconnect()
            self.get_logger().info("Shutting down after successful send.")
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Failed to send request: {e}")
            self.client.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = SocketClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()






# import cv2
# import rclpy
# from rclpy.node import Node
# from PIL import Image
# import numpy as np
# import matplotlib.pyplot as plt

# from .utils.my_custom_socket import MyClient


# class SocketClientNode(Node):
#     def __init__(self):
#         super().__init__("socket_client_node")
#         self.client = MyClient(host="127.0.0.1", port=65432)
#         self.timer = self.create_timer(1.0, self.timer_callback)
#         self.send_once = False

#     def timer_callback(self):
#         # Read an image
#         if self.send_once:
#             return
        
#         client_connected = self.client.connect()
#         if not client_connected:
#             self.get_logger().error(f"Server not started")
#             return
        
#         mask_path = '/home/bham/Desktop/test_scene/cereal.png'
#         mask_image = Image.open('/home/bham/Desktop/test_scene/cereal.png').convert("1")
#         mask = np.array(mask_image, dtype=np.uint8)

#         if mask is None:
#             self.get_logger().error(f"[Client] Failed to read mask at {mask_path}")
#             return

#         object_name = "gorilla"
#         try:
#             result = self.client.request_msg(
#                 msg_type_in=["image2d", "string"],
#                 msg_in=[mask, object_name],)
#             self.get_logger().info(f"Response: {result}")
#             self.sent_once = True
#         except Exception as e:
#             self.get_logger().error(f"Failed to send request: {e}")

#         self.client.disconnect()


# def main(args=None):
#     rclpy.init(args=args)
#     node = SocketClientNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()