import RPi.GPIO as GPIO
import time
import serial
import threading

# GPIO 핀 설정
servo_pin = 4  # 서보모터를 연결할 GPIO 핀 번호
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.05)  # 거리 센서 통신 설정

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM 주파수 설정 (서보모터의 주파수는 보통 50Hz)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

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

# 모터 초기화
motors = [
    MotorController(18, 17, 27),  # motor1: left front
    MotorController(22, 23, 24),  # motor2: right front
    MotorController(9, 10, 11),   # motor3: left back
    MotorController(25, 8, 7),    # motor4: right back
]

# 서보모터 제어 함수
def set_angle(angle):
    """서보모터의 각도를 설정"""
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.2)  # 서보모터가 각도 변경 후 움직일 시간을 줌

# 거리 측정 함수
def read_distance():
    """TF Luna 센서에서 거리 데이터를 읽어옴"""
    ser.reset_input_buffer()  # 입력 버퍼 초기화
    ser.write(b'\x42\x57\x02\x00\x00\x00\x01\x06')  # 데이터 요청 명령
    response = ser.read(9)
    if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
        distance = response[2] + response[3] * 256
        return distance
    else:
        return None

# 장애물 감지 함수
def is_obstacle_ahead(threshold=30):
    """전방에 장애물이 있는지 확인"""
    set_angle(90)  # 서보모터를 전방으로 설정
    distance = read_distance()
    if distance is not None and distance <= threshold:
        return True
    else:
        return False

def find_clear_direction(threshold=30):
    """장애물이 없는 방향을 탐색"""
    # 좌측 확인
    set_angle(180)
    left_distance = read_distance()
    # 우측 확인
    set_angle(0)
    right_distance = read_distance()

    # 장애물이 없는 방향 반환
    if left_distance is not None and left_distance > threshold:
        return 'left'
    elif right_distance is not None and right_distance > threshold:
        return 'right'
    else:
        return None

# 장애물 회피 함수
def avoid_obstacle(direction):
    """장애물을 회피하는 동작"""
    if direction == 'left':
        # 좌회전
        motors[0].backward(30)
        motors[1].forward(30)
        motors[2].backward(30)
        motors[3].forward(30)
    elif direction == 'right':
        # 우회전
        motors[0].forward(30)
        motors[1].backward(30)
        motors[2].forward(30)
        motors[3].backward(30)
    
    time.sleep(0.5)  # 일정 시간 회전 후 정지
    for motor in motors:
        motor.stop()

# 로봇 전진 함수
def move_forward(speed=40):
    """로봇 전진"""
    for motor in motors:
        motor.forward(speed)

# 로봇 정지 함수
def stop_robot():
    """로봇 정지"""
    for motor in motors:
        motor.stop()

# 장애물 감지 및 회피 로직
def main():
    try:
        while True:
            if is_obstacle_ahead(threshold=30):
                print("장애물 감지! 회피 동작 수행")
                direction = find_clear_direction()
                if direction:
                    avoid_obstacle(direction)
                else:
                    print("회피할 수 있는 방향이 없습니다. 로봇을 정지합니다.")
                    stop_robot()
                    break
            else:
                print("전진 중...")
                move_forward()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        stop_robot()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
