import os
import sys
import math

sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar

def update_scan(scan_data, laser):
    r = laser.doProcessSimple(scan_data)
    if r:
        left_angle = None
        left_distance = float('inf')  # 초기값을 무한대 거리로 설정
        right_angle = None
        right_distance = float('inf')  # 초기값을 무한대 거리로 설정

        for point in scan_data.points:
            degree_angle = math.degrees(point.angle)
            if -30 <= degree_angle <= 30:  # -30도에서 +30도
                if 0.01 <= point.range <= 8.0:  # 거리 값이 0.01m ~ 8m 사이일 때만 유효한 데이터로 간주
                    # 좌측 감지
                    if degree_angle < 0 and point.range < left_distance:
                        left_angle = degree_angle  # 좌측 각도
                        left_distance = point.range  # 좌측 거리

                    # 우측 감지
                    if degree_angle > 0 and point.range < right_distance:
                        right_angle = degree_angle  # 우측 각도
                        right_distance = point.range  # 우측 거리

                    # 0도 근처 확인
                    if -1 <= degree_angle <= 1:  # 0도 주변 데이터 출력
                        print(f"0도 근처 데이터 -> 각도: {degree_angle:.2f}°, 거리: {point.range:.2f} meters")

        if left_angle is not None:
            print(f"좌측 감지 -> 각도: {left_angle:.2f}°, 거리: {left_distance * 100:.2f} cm")
        else:
            print("좌측에서 감지된 물체가 없습니다.")

        if right_angle is not None:
            print(f"우측 감지 -> 각도: {right_angle:.2f}°, 거리: {right_distance * 100:.2f} cm")
        else:
            print("우측에서 감지된 물체가 없습니다.")

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
    # TYPE_TRIANGLE으로 변경
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
