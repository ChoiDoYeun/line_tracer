import os
import sys
sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar

if __name__ == "__main__":
    ydlidar.os_init()  # 초기화
    laser = ydlidar.CYdLidar()
    ports = ydlidar.lidarPortList()
    
    # 사용 가능한 포트를 가져오기
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
    
    # X2 모델의 경우 단방향 통신이므로 SingleChannel을 True로 설정
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    # 초기화
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan = ydlidar.LaserScan()
        
        # 스캔 데이터를 반복해서 수집
        while ret and ydlidar.os_isOk():
            r = laser.doProcessSimple(scan)
            if r:
                print(f"Scan received[{scan.stamp}]: {scan.points.size()} ranges at {1.0/scan.config.scan_time}Hz")
            else:
                print("Failed to get Lidar Data.")
        
        # Lidar 종료
        laser.turnOff()
    laser.disconnecting()
