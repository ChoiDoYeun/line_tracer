import os
import sys
import math
import RPi.GPIO as GPIO
import time

sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar

# MotorController 클래스
class MotorController:
    def __init__(self, en, in1, in2):
        self.en = en
        self.in1 = in1
        self.in2 = in2
        GPIO.setup(self.en, GPIO.OUT)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm = GPIO.PWM(self.en, 100)
        self.pwm.start(0)

    def set_speed(self, speed):
        speed = max(min(speed, 60), -60)
        self.pwm.ChangeDutyCycle(abs(speed))

    def forward(self, speed=40):
        self.set_speed(speed)
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

    def backward(self, speed=40):
        self.set_speed(speed)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)

    def stop(self):
        self.set_speed(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup([self.en, self.in1, self.in2])

# YDLidar에서 거리 데이터를 가져오는 함수
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

        # 디버깅 정보 출력
        print(f"정면 거리: {front_distance * 100:.2f} cm, 좌측 거리: {left_distance * 100:.2f} cm, 우측 거리: {right_distance * 100:.2f} cm")
        
        return front_distance, left_distance, right_distance
    else:
        print("Failed to get Lidar Data.")
        return None, None, None

def dynamic_turn(left_motor, right_motor, left_speed, right_speed, direction):
    """모터 속도를 기반으로 동적 회전"""
    if direction == "right":  # 우회전
        left_motor.forward(left_speed)
        right_motor.forward(right_speed)
        print(f"우회전: 좌측 속도 {left_speed}, 우측 속도 {right_speed}")
    elif direction == "left":  # 좌회전
        left_motor.backward(left_speed)
        right_motor.forward(right_speed)
        print(f"좌회전: 좌측 속도 {left_speed}, 우측 속도 {right_speed}")

if __name__ == "__main__":
    # GPIO 설정
    GPIO.setmode(GPIO.BCM)

    # 모터 초기화
    motor1 = MotorController(18, 17, 27)  # left front
    motor2 = MotorController(22, 23, 24)  # right front
    motor3 = MotorController(9, 10, 11)   # left back
    motor4 = MotorController(25, 8, 7)    # right back

    # YDLidar 설정
    ydlidar.os_init()
    laser = ydlidar.CYdLidar()
    ports = ydlidar.lidarPortList()

    port = "/dev/ttyUSB0"
    for key, value in ports.items():
        port = value

    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 7.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3000)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    # Lidar 초기화
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan_data = ydlidar.LaserScan()

        try:
 
