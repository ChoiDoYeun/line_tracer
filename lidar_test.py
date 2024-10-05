import sys
from ydlidar import *

def main():
    laser = CYdLidar()
    laser.setlidaropt(LidarPropSerialPort, "/dev/ttyUSB0")
    laser.setlidaropt(LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(LidarPropLidarType, TYPE_TRIANGLE)
    laser.setlidaropt(LidarPropDeviceType, YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(LidarPropScanFrequency, 7.0)

    if laser.initialize():
        print("YDLidar initialized successfully!")
        while True:
            scan = laser.doProcessSimple(scan=None)
            if scan:
                print("Lidar scan received:", len(scan))
    else:
        print("Failed to initialize YDLidar.")
    
    laser.turnOff()
    laser.disconnecting()

if __name__ == "__main__":
    main()
