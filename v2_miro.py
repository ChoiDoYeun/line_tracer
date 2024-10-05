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
        # 섹터 설정
        sector_size = 45  # 섹터 당 각도 (예: 45도)
        num_sectors = int(360 / sector_size)
        sectors = [float('inf')] * num_sectors  # 각 섹터의 최소 거리 초기화

        for point in scan_data.points:
            degree_angle = math.degrees(point.angle)
            if degree_angle < 0:
                degree_angle += 360  # 각도를 0~360도로 변환

            if 0.01 <= point.range <= 8.0:
                # 해당 포인트가 속한 섹터 계산
                sector_index = int(degree_angle / sector_size) % num_sectors
                # 해당 섹터의 최소 거리 업데이트
                sectors[sector_index] = min(sectors[sector_index], point.range)

        # 섹터별 거리 정보 디버깅 출력
        for i, dist in enumerate(sectors):
            print(f"섹터 {i}: 거리 {dist * 100:.2f} cm")

        return sectors
    else:
        print("라이다 데이터 수집 실패.")
        return None

def dynamic_turn(left_motor, right_motor, left_speed, right_speed, direction):
    """모터 속도를 기반으로 동적 회전"""
    if direction == "right":  # 우회전
        left_motor.forward(left_speed)
        right_motor.backward(right_speed)
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
    # 좌측 및 우측 모터 그룹화
    left_motors = [motor1, motor3]
    right_motors = [motor2, motor4]

    try:
        while True:
            sectors = update_scan(scan_data, laser)

            if sectors is None:
                continue

            # 섹터 인덱스 설정 (예: 8개의 섹터일 경우)
            front_sector = 0  # 0도 ~ 45도
            right_sector = 2  # 90도 근처
            left_sector = 6   # 270도 근처

            # 각 방향의 최소 거리 가져오기
            front_dist = sectors[front_sector]
            right_dist = sectors[right_sector]
            left_dist = sectors[left_sector]

            # 거리 임계값 설정 (필요에 따라 조정)
            threshold_front = 0.6  # 60cm
            threshold_side = 0.15  # 15cm

            # 우수법 로직 적용
            if right_dist > threshold_side:
                # 우측이 열려 있으므로 우회전
                print("우측이 열려 있음, 우회전 중...")
                for motor in left_motors:
                    motor.forward(40)
                for motor in right_motors:
                    motor.backward(40)
            elif front_dist > threshold_front:
                # 전방이 열려 있으므로 전진
                print("전방이 열려 있음, 전진 중...")
                for motor in left_motors + right_motors:
                    motor.forward(40)
            else:
                # 전방과 우측이 막혀 있으므로 좌회전
                print("전방 및 우측이 막혀 있음, 좌회전 중...")
                for motor in left_motors:
                    motor.backward(40)
                for motor in right_motors:
                    motor.forward(40)

            # 잠시 대기
            time.sleep(0.1)

 except KeyboardInterrupt:
    # 정지 및 정리
    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()
    motor1.cleanup()
    motor2.cleanup()
    motor3.cleanup()
    motor4.cleanup()
    
    # Lidar 종료
    laser.turnOff()
    laser.disconnecting()
