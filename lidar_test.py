import PyLidar3
import time

port = "/dev/ttyUSB0"
lidar = PyLidar3.YdLidarX4(port)

if lidar.Connect():
    print("LiDAR 연결 성공")
    gen = lidar.StartScanning()
    start_time = time.time()
    
    while (time.time() - start_time) < 30:  # 30초 동안 스캔
        data = next(gen)
        print(data)
    
    lidar.StopScanning()
    lidar.Disconnect()
else:
    print("LiDAR 연결 실패")
