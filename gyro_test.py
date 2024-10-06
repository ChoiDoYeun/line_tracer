import RPi.GPIO as GPIO
import time
import adafruit_adxl34x
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

# Y축 가속도 데이터를 사용해 각도 계산 함수
def get_angle():
    accel_data = accelerometer.acceleration
    x, y, z = accel_data
    angle = math.degrees(math.atan2(x, y))  # X, Y축으로 각도 계산
    return angle

# 45도 회전 함수
def rotate_to_angle(target_angle):
    current_angle = get_angle()
    while abs(current_angle - target_angle) > 1:  # 목표 각도에 도달할 때까지 회전
        if current_angle < target_angle:
            control_motors(-50, 50)  # 좌회전
        else:
            control_motors(50, -50)  # 우회전
        time.sleep(0.1)  # 자주 업데이트하여 각도를 추적
        current_angle = get_angle()

    control_motors(0, 0)  # 회전 후 정지

try:
    # 45도 회전
    rotate_to_angle(45)
finally:
    motor1.cleanup()
    motor2.cleanup()
    motor3.cleanup()
    motor4.cleanup()
    GPIO.cleanup()
