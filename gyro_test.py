import RPi.GPIO as GPIO
import time
import adafruit_adxl34x
import board
import busio
import math

# PID 제어 상수 설정
Kp = 1.00  # 비례 상수
Ki = 0.00  # 적분 상수
Kd = 0.0   # 미분 상수

# 자이로 센서 초기화
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c, address=0x1D)

# MotorController 클래스 (기존 코드 동일)
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
        speed = max(min(speed, 50), -50)
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

# 모터 컨트롤러 초기화
motor1 = MotorController(18, 17, 27)
motor2 = MotorController(22, 23, 24)
motor3 = MotorController(9, 10, 11)
motor4 = MotorController(25, 8, 7)

def control_motors(left_speed, right_speed):
    if left_speed >= 0:
        motor1.forward(left_speed)
        motor3.forward(left_speed)
    else:
        motor1.backward(-left_speed)
        motor3.backward(-left_speed)

    if right_speed >= 0:
        motor2.forward(right_speed)
        motor4.forward(right_speed)
    else:
        motor2.backward(-right_speed)
        motor4.backward(-right_speed)

# 현재 각도를 추적하기 위한 변수 초기화
current_angle = 0
last_time = time.time()

def get_current_angle():
    global current_angle, last_time
    now = time.time()
    dt = now - last_time
    last_time = now
    # 자이로 센서에서 Z축 각속도 읽기 (라디안/초)
    gyro_z = imu.gyro[2]
    # 각속도를 적분하여 각도 계산 (도 단위로 변환)
    current_angle += math.degrees(gyro_z * dt)
    return current_angle

def rotate_to_angle(target_angle):
    angle_tolerance = 5  # 허용 오차 (도)
    speed = 50  # 모터 속도
    current_angle = get_current_angle()
    while abs(current_angle - target_angle) > angle_tolerance:
        if current_angle < target_angle:
            # 시계 방향 회전
            control_motors(left_speed=speed, right_speed=-speed)
        else:
            # 반시계 방향 회전
            control_motors(left_speed=-speed, right_speed=speed)
        time.sleep(0.01)  # 작은 딜레이
        current_angle = get_current_angle()
    # 목표 각도에 도달하면 모터 정지
    control_motors(0, 0)

# 예제 사용법: 로봇을 90도 회전시키기
try:
    rotate_to_angle(90)
finally:
    # 프로그램 종료 시 리소스 정리
    motor1.cleanup()
    motor2.cleanup()
    motor3.cleanup()
    motor4.cleanup()
    GPIO.cleanup()
