import cv2

from my_custom_socket import MyClient

if __name__ == "__main__":
    # Init
    client = MyClient(host="127.0.0.1", port=65432)
    client.start()

    while True:
        string_input = input("Type and enter to send: ")

        my_image = cv2.imread("example.png")

        # Request
        result = client.request_msg(
            msg_type_in=["image3d", "image3d", "image3d", "string"],
            msg_in=[my_image, my_image, my_image, string_input],
        )
        print(result)
