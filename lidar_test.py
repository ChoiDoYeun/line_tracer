import os
import sys
import math
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar

def update_scan(scan_data, laser):
    r = laser.doProcessSimple(scan_data)
    if r:
        angles = []
        distances = []
        for point in scan_data.points:
            # 각도를 -30도(-0.5236 라디안)에서 +30도(0.5236 라디안) 사이로 제한
            if -0.5236 <= point.angle <= 0.5236:
                angles.append(point.angle)
                distances.append(point.range)
        return angles, distances
    else:
        print("Failed to get Lidar Data.")
        return [], []

def animate(frame, laser, scan_data, plot):
    angles, distances = update_scan(scan_data, laser)
    plot.set_offsets([(math.cos(a) * d, math.sin(a) * d) for a, d in zip(angles, distances)])
    return plot,

if __name__ == "__main__":
    ydlidar.os_init()  # SDK 초기화
    laser = ydlidar.CYdLidar()
    ports = ydlidar.lidarPortList()

    # 포트 설정
    port = "/dev/ttyUSB0"
    for key, value in ports.items():
        port = value

    # Lidar 설정
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 20)

    # 단방향 통신 설정
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    # Lidar 초기화
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan_data = ydlidar.LaserScan()

        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
        plot = ax.scatter([], [], s=5)

        ax.set_ylim(0, 10)  # 거리의 범위를 미터 단위로 설정 (0~10m)
        ani = FuncAnimation(fig, animate, fargs=(laser, scan_data, plot), interval=100)

        plt.show()

        # Lidar 종료
        laser.turnOff()
    laser.disconnecting()
