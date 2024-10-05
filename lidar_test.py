import os
import sys
sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar

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
        scan = ydlidar.LaserScan()

        # 스캔 데이터 수집
        while ret and ydlidar.os_isOk():
            r = laser.doProcessSimple(scan)
            if r:
                scan_time = scan.config.scan_time if scan.config.scan_time != 0 else 1.0
                print(f"Scan received[{scan.stamp}]: {scan.points.size()} ranges at {1.0 / scan_time}Hz")
                
                # 각 포인트의 거리 출력
                for point in scan.points:
                    print(f"Angle: {point.angle:.2f} rad, Distance: {point.range:.2f} meters")
            else:
                print("Failed to get Lidar Data.")

        # Lidar 종료
        laser.turnOff()
    laser.disconnecting()
