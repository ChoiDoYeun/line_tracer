import RPi.GPIO as GPIO
import time

# MotorController 클래스 정의
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

try:
    # 각 모터를 2초 동안 정방향으로 구동
    print("Motor 1 forward")
    motor1.forward(50)
    time.sleep(2)
    motor1.stop()
    
    print("Motor 2 forward")
    motor2.forward(50)
    time.sleep(2)
    motor2.stop()
    
    print("Motor 3 forward")
    motor3.forward(50)
    time.sleep(2)
    motor3.stop()
    
    print("Motor 4 forward")
    motor4.forward(50)
    time.sleep(2)
    motor4.stop()
    
    # 각 모터를 2초 동안 역방향으로 구동
    print("Motor 1 backward")
    motor1.backward(50)
    time.sleep(2)
    motor1.stop()
    
    print("Motor 2 backward")
    motor2.backward(50)
    time.sleep(2)
    motor2.stop()
    
    print("Motor 3 backward")
    motor3.backward(50)
    time.sleep(2)
    motor3.stop()
    
    print("Motor 4 backward")
    motor4.backward(50)
    time.sleep(2)
    motor4.stop()

finally:
    # GPIO 정리
    motor1.cleanup()
    motor2.cleanup()
    motor3.cleanup()
    motor4.cleanup()
    GPIO.cleanup()
