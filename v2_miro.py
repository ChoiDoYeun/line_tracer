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

        try:
            while True:
                # 거리 데이터 업데이트
                front_dist, left_dist, right_dist = update_scan(scan_data, laser)

                if front_dist is None or left_dist is None or right_dist is None:
                    continue

                # 정면과 좌측, 우측 거리 기준으로 행동 결정
                if front_dist <= 0.6:  # 정면에 벽이 45cm 이내일 때 우회전 준비
                    print("정면에 벽 감지, 우회전 준비 중...")
                    motor1.stop()
                    motor2.stop()
                    motor3.stop()
                    motor4.stop()

                    # 우회전하며 정면이 1m 이상이 될 때까지 회전
                    while front_dist <= 2.0:
                        print("우회전 중...")
                        dynamic_turn(motor1, motor2, 40, 40, "right")
                        dynamic_turn(motor3, motor4, 40, 40, "right")
                        time.sleep(0.001)  # 잠시 대기 후 정면 거리 업데이트
                        front_dist, _, _ = update_scan(scan_data, laser)

                elif left_dist <= 0.15:  # 좌측에 벽이 30cm 이내일 때 우회전
                    print("좌측에 벽이 가까움, 우회전 중...")
                    motor1.stop()
                    motor2.stop()
                    motor3.stop()
                    motor4.stop()

                    # 우회전
                    dynamic_turn(motor1, motor2, 40, 40, "right")
                    dynamic_turn(motor3, motor4, 40, 40, "right")
                    time.sleep(0.1)

                elif right_dist <= 0.15:  # 우측에 벽이 30cm 이내일 때 좌회전
                    print("우측에 벽이 가까움, 좌회전 중...")
                    motor1.stop()
                    motor2.stop()
                    motor3.stop()
                    motor4.stop()

                    # 좌회전
                    dynamic_turn(motor1, motor2, 40, 40, "left")
                    dynamic_turn(motor3, motor4, 40, 40, "left")
                    time.sleep(0.1)

                else:  # 정면에 장애물이 없으면 전진
                    motor1.forward()
                    motor2.forward()
                    motor3.forward()
                    motor4.forward()
                    print("전진 중")

                # 잠시 대기
                time.sleep(0.001)

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
