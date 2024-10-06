import RPi.GPIO as GPIO
import time
from adafruit_adxl34x import ADXL345
import board
import busio
import math

# 자이로 센서 초기화
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c, address=0x1D)

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
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)

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

def get_angle():
    accel_data = accelerometer.acceleration
    x, y, z = accel_data
    angle = math.degrees(math.atan2(y, x))  # X와 Y축을 이용해 각도 계산
    return angle

def rotate_to_angle(target_angle):
    current_angle = get_angle()
    while abs(current_angle - target_angle) > 1:  # 목표 각도에 도달할 때까지 회전
        if current_angle < target_angle:
            control_motors(-30, 30)  # 좌측 회전
        else:
            control_motors(30, -30)  # 우측 회전
        time.sleep(0.1)
        current_angle = get_angle()

    control_motors(0, 0)  # 회전 후 정지

# 45도 회전
rotate_to_angle(45)
