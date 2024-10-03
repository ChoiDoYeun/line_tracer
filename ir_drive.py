import RPi.GPIO as GPIO
import time

# GPIO 핀 번호 설정
RIGHT_SENSOR_PIN = 21  # 우측 IR 센서 GPIO 핀
FRONT_SENSOR_PIN = 20  # 앞쪽 IR 센서 GPIO 핀

# GPIO 모드 및 핀 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(RIGHT_SENSOR_PIN, GPIO.IN)
GPIO.setup(FRONT_SENSOR_PIN, GPIO.IN)

# MotorController 클래스 (이전 코드 그대로 사용)
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

# 모터 초기화 (4개의 모터 제어)
motor1 = MotorController(18, 17, 27)  # left front
motor2 = MotorController(22, 23, 24)  # right front
motor3 = MotorController(9, 10, 11)   # left back
motor4 = MotorController(25, 8, 7)    # right back

# 우측 벽이 감지될 때 직진하고, 벽이 감지되지 않으면 우회전하는 로직
try:
    while True:
        right_sensor_value = GPIO.input(RIGHT_SENSOR_PIN)  # 우측 센서 값 읽기
        front_sensor_value = GPIO.input(FRONT_SENSOR_PIN)  # 전방 센서 값 읽기

        if right_sensor_value == 0:
            # 우측에 벽이 감지되면 직진
            motor1.forward(50)
            motor2.forward(50)
            motor3.forward(50)
            motor4.forward(50)
            print("우측에 벽이 감지됨: 직진")
        else:
            # 우측에 벽이 없으면 우회전
            motor1.forward(30)  # 좌측 바퀴는 그대로
            motor2.backward(30)  # 우측 바퀴는 후진하여 우회전
            motor3.forward(30)
            motor4.backward(30)
            print("우측에 벽이 없음: 우회전")

except KeyboardInterrupt:
  motor1.cleanup()
  motor2.cleanup()
  motor3.cleanup()
  motor4.cleanup()
  GPIO.cleanup()
