import RPi.GPIO as GPIO
import time
import serial
import threading

# GPIO 핀 설정
servo_pin = 4  # 서보모터를 연결할 GPIO 핀 번호
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.01)  # 거리 센서 통신 설정

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM 주파수 설정 (서보모터의 주파수는 보통 50Hz)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

# MotorController 클래스 정의 (이전 코드와 동일)
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
    # 서보모터가 이동하는 동안 프로그램이 멈추지 않도록 대기 시간을 제거하거나 최소화

# 거리 측정 함수
def read_distance():
    """TF Luna 센서에서 거리 데이터를 읽어옴"""
    response = ser.read(9)
    if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
        distance = response[2] + response[3] * 256
        return distance
    else:
        return None

# 전역 변수 설정
front_distance = None
left_distance = None
right_distance = None

# 센서 데이터를 지속적으로 읽어오는 스레드 함수
def sensor_read_thread():
    global front_distance, left_distance, right_distance
    angle_positions = [90, 45, 135]  # 전방, 우측, 좌측 각도
    index = 0
    while True:
        angle = angle_positions[index]
        set_angle(angle)
        time.sleep(0.05)  # 최소한의 대기 시간
        distance = read_distance()
        if angle == 90:
            front_distance = distance
        elif angle == 45:
            right_distance = distance
        elif angle == 135:
            left_distance = distance
        index = (index + 1) % len(angle_positions)
        # 각도 변경 후 바로 다음 측정으로 이동

# 장애물 감지 함수
def is_obstacle_ahead(threshold=60):
    """전방에 장애물이 있는지 확인"""
    if front_distance is not None and front_distance <= threshold:
        return True
    else:
        return False

def find_clear_direction(threshold=30):
    """장애물이 없는 방향을 탐색"""
    # 좌우 거리 비교
    if left_distance is not None and left_distance > threshold:
        return 'left'
    elif right_distance is not None and right_distance > threshold:
        return 'right'
    else:
        return None

# 장애물 회피 함수
def avoid_obstacle(direction):
    """장애물을 회피하는 동작"""
    print(f"{direction} 방향으로 회피합니다.")
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

# 메인 함수
def main():
    # 센서 읽기 스레드 시작
    sensor_thread = threading.Thread(target=sensor_read_thread)
    sensor_thread.daemon = True
    sensor_thread.start()

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
            time.sleep(0.05)  # 메인 루프 주기 설정
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        stop_robot()
        pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
