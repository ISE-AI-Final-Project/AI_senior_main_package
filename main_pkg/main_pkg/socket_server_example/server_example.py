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
            recv_int1, recv_int2, recv_string = recv_msg
            mean_val = recv_int1 + recv_int2
            print(f"[{server.server_name}] Recv String value: {recv_string}")
            print(f"[{server.server_name}] Mean pixel value: {mean_val:.2f}")

            # Send response
            server.send_response(msg_type_out=["float"], msg_out=[mean_val])
        else:
            print(f"[{server.server_name}] Connection Lost. Restarting.")
            server.restart()
