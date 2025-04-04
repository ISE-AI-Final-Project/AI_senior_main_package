import numpy as np
from my_custom_socket import MyServer

if __name__ == "__main__":
    # Init
    server = MyServer(host="127.0.0.1", port=65432)
    server.start()

    while True:
        # Wait for msg
        recv_msg = server.wait_for_msg()
        if recv_msg is not None:
            rgb, depth, recv_string = recv_msg
            # print(f"[{server.server_name}] Recv String value: {recv_string}")
            # print(f"[{server.server_name}] Mean pixel value: {mean_val:.2f}")

            print(rgb.shape, rgb.dtype)
            print(depth.shape, depth.dtype)
            print(recv_string)


            # Send response
            server.send_response(msg_type_out=["float"], msg_out=[12.3])

            print(f"[{server.server_name}] Response Sent. Restarting.")
            server.restart()
        else:
            print(f"[{server.server_name}] Connection Lost. Restarting.")
            server.restart()
