import os
import sys
sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar
import time

def main():
    # LiDAR 초기화
    lidar = CYdLidar()

    # LiDAR 설정
    port = "/dev/ttyUSB0"  # 포트 설정
    lidar.setlidaropt(CYdLidar.LidarPropSerialPort, port)
    lidar.setlidaropt(CYdLidar.LidarPropSerialBaudrate, 115200)
    lidar.setlidaropt(CYdLidar.LidarPropLidarType, 1)  # Type 1: X2, X4, etc.
    lidar.setlidaropt(CYdLidar.LidarPropDeviceType, 0)
    lidar.setlidaropt(CYdLidar.LidarPropSampleRate, 5)
    lidar.setlidaropt(CYdLidar.LidarPropScanFrequency, 7.0)

    # LiDAR 연결 및 시작
    if not lidar.initialize():
        print("LiDAR 초기화 실패")
        return

    if not lidar.turnOn():
        print("LiDAR 시작 실패")
        return

    print("LiDAR가 시작되었습니다. Ctrl+C를 눌러 종료하십시오.")

    try:
        while True:
            # 스캔 데이터 가져오기
            scan = lidar.doProcessSimple(scan_time=0.1)
            if scan:
                for point in scan.points:
                    # 각도와 거리 정보 출력 (거리: meter 단위)
                    print(f"Angle: {point.angle}, Distance: {point.range}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Ctrl+C로 종료
        print("LiDAR 종료 중...")

    # LiDAR 종료
    lidar.turnOff()
    lidar.disconnecting()

if __name__ == "__main__":
    main()
