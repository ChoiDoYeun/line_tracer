import RPi.GPIO as GPIO
import time
import adafruit_adxl34x
import board
import busio
import math

# 자이로 센서 초기화
i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_adxl34x.ADXL345(i2c, address=0x1D)

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

# 각도 계산 함수
def get_current_angle():
    accel_x, accel_y, accel_z = imu.acceleration
    # X, Y 축의 가속도 값을 통해 각도를 계산 (예: 피치 각도)
    pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
    return pitch

# 회전 각도 제어 함수 (피드백 제어)
def turn_to_angle(target_angle, speed=30, tolerance=2):
    current_angle = get_current_angle()
    
    while abs(target_angle - current_angle) > tolerance:
        error = target_angle - current_angle
        
        # PID 제어로 속도 조정 가능 (간단히 proportional 제어)
        turn_speed = error * 0.5  # P 제어 계수 0.5로 설정
        
        if turn_speed > 0:
            control_motors(turn_speed, -turn_speed)  # 좌회전
        else:
            control_motors(-turn_speed, turn_speed)  # 우회전
        
        current_angle = get_current_angle()  # 각도 업데이트
        time.sleep(0.1)
    
    control_motors(0, 0)  # 회전 후 모터 정지

# 예시: 90도 회전
turn_to_angle(90)
