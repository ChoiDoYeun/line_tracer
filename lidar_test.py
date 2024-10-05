import os
import sys
import math

sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar

def update_scan(scan_data, laser):
    r = laser.doProcessSimple(scan_data)
    if r:
        front_distance = float('inf')  # 정면 거리
        left_distance = float('inf')   # 좌측 거리
        right_distance = float('inf')  # 우측 거리

        for point in scan_data.points:
            degree_angle = math.degrees(point.angle)  # 각도를 도 단위로 변환
            if 0.01 <= point.range <= 8.0:  # 유효한 거리 데이터(0.01m ~ 8m)

                # 정면 (0도)
                if -1 <= degree_angle <= 1:  # 0도 근처
                    front_distance = min(front_distance, point.range)

                # 우측 (90도)
                if 89 <= degree_angle <= 91:  # 90도 근처 (우측)
                    right_distance = min(right_distance, point.range)

                # 좌측 (-90도)
                if -91 <= degree_angle <= -89:  # -90도 근처 (좌측)
                    left_distance = min(left_distance, point.range)

        # 거리 출력 (cm로 변환하여 출력)
        print(f"정면 거리: {front_distance * 100:.2f} cm" if front_distance != float('inf') else "정면에서 감지된 물체가 없습니다.")
        print(f"우측 거리: {right_distance * 100:.2f} cm" if right_distance != float('inf') else "우측에서 감지된 물체가 없습니다.")
        print(f"좌측 거리: {left_distance * 100:.2f} cm" if left_distance != float('inf') else "좌측에서 감지된 물체가 없습니다.")
    else:
        print("Failed to get Lidar Data.")

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
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 7.0)  # 권장 주파수로 설정
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3000)  # 샘플레이트 유지

    # 단방향 통신 설정
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    # Lidar 초기화
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan_data = ydlidar.LaserScan()

        # 거리 데이터 출력
        update_scan(scan_data, laser)

        # Lidar 종료
        laser.turnOff()
    laser.disconnecting()
