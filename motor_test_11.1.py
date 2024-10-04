import RPi.GPIO as GPIO
import random
import time

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
        # 속도를 -100에서 100 사이로 제한
        speed = max(min(speed, 100), -100)
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

# GPIO 설정
GPIO.setmode(GPIO.BCM)

# 모터 초기화 (핀 번호는 사용 중인 모터에 맞게 설정)
motor1 = MotorController(18, 17, 27) # left front
motor2 = MotorController(22, 23, 24) # right front
motor3 = MotorController(9, 10, 11)  # left back
motor4 = MotorController(25, 8, 7)   # right back

def control_motors(left_speed, right_speed):
    # 속도 범위 제한 (리밋)
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

  # 랜덤한 값을 모터에 넣어주는 함수
def random_motor_control():
    try:
        while True:
            # -100에서 100 사이의 랜덤한 속도를 생성
            left_speed = random.randint(-100, 100)
            right_speed = random.randint(-100, 100)

            # 모터 속도 제어
            control_motors(left_speed, right_speed)

            # 현재 속도를 출력 (테스트를 위해)
            print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")

            # 0.5초 대기
            time.sleep(0.5)

    except KeyboardInterrupt:
        # 사용자가 Ctrl+C를 눌러 중지할 때 모터 정지 및 GPIO 클린업
        motor1.cleanup()
        motor2.cleanup()
        motor3.cleanup()
        motor4.cleanup()
        print("모터 제어를 중지합니다.")

# 랜덤 모터 제어 시작
random_motor_control()
