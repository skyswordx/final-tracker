import setup_path
import airsim
import numpy as np
import cv2
import time

def main():
    client = airsim.MultirotorClient(ip="192.168.1.121")
    client.confirmConnection()
    # print("Connected to AirSim ")
    client.enableApiControl(True)
    client.armDisarm(True)

    frame_count = 0
    start_time = time.time()

    try:
        print("开始实时显示图像...")
        while True:
            # 只请求场景视觉图像
            response = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]

            # 将图像数据转换为numpy数组，以便使用opencv进行显示
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)

            # 计算并显示帧率
            frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time > 1:
                fps = frame_count / elapsed_time
                print("FPS: {:.2f}".format(fps))
                frame_count = 0
                start_time = time.time()

            # 使用opencv显示图像
            cv2.imshow("Scene Image", img_rgb)
            
            # 设置延迟以达到大约60Hz的更新频率
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        print("重置到原始状态...")
        client.armDisarm(False)
        client.reset()
        client.enableApiControl(False)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
