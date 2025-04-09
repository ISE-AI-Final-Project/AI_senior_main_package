import numpy as np
from my_custom_socket import MyServer
import matplotlib.pyplot as plt

if __name__ == "__main__":
    server = MyServer(host="127.0.0.1", port=65432)
    server.start()

    while True:
        recv_msg = server.wait_for_msg()
        if recv_msg is not None:
            recv_mask, recv_string = recv_msg
            print(f"[{server.server_name}] Received string: {recv_string}")
            print(f"[{server.server_name}] Received mask shape: {recv_mask.shape}")

            string_res = 'Received : {0}'.format(recv_string)

            shape0 = recv_mask.shape[0]
            plt.imshow(recv_mask, cmap = 'gray')
            plt.show()
            server.send_response(msg_type_out=["float", "string"], msg_out=[shape0, string_res])
        else:
            pass
            # print(f"[{server.server_name}] Connection Lost. Restarting.")
