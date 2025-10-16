import socket
import cv2
import numpy as np
from PIL import Image
import io
import time

# 绑定的IP和端口
IP = "127.0.0.1"
PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((IP, PORT))

print(f"Listening on {IP}:{PORT}")

last_time = time.time()
while True:
    data, addr = sock.recvfrom(65536)  # 接收数据

    # 计算FPS
    current_time = time.time()
    fps = 1.0 / (current_time - last_time)
    print(f"FPS: {fps:.2f}")
    last_time = current_time

    try:
        # 将接收的数据转换为一个字节流
        img_data = io.BytesIO(data)

        # 从这个字节流中读取图像
        image = np.array(Image.open(img_data))

        # OpenCV需要BGR格式，而Pillow读取图像格式为RGB，所以需要转换
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # 使用OpenCV显示图像
        cv2.imshow('Received Image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 添加退出条件
            break

    except Exception as e:
        print(f"Error: {e}")

cv2.destroyAllWindows()
