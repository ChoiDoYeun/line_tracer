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

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup([self.en, self.in1, self.in2])

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
    # 각도 변경 후 대기 시간 조정
    time.sleep(0.2)

# 거리 측정 함수
def read_distance():
    """TF Luna 센서에서 거리 데이터를 읽어옴"""
    response = ser.read(9)
    if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
        distance = response[2] + response[3] * 256
        return distance
    else:
        return None

# 최신 거리 값을 저장할 전역 변수
latest_distance = None

# 센서 데이터를 지속적으로 읽어오는 스레드 함수
def sensor_read_thread():
    global latest_distance
    while True:
        distance = read_distance()
        if distance is not None:
            latest_distance = distance
        time.sleep(0.01)  # 너무 빈번한 읽기를 방지

# 센서를 연속 출력 모드로 설정
def initialize_sensor():
    """센서를 연속 출력 모드로 설정"""
    ser.write(b'\x42\x57\x02\x00\x00\x00\x00\xff')  # 연속 모드 설정 명령
    time.sleep(0.1)

# 전방 거리 확인 및 멈춤 로직
def check_front_and_stop():
    """전방 거리를 확인하고, 장애물이 있으면 우회전 시도"""
    front_distance = latest_distance

    if front_distance is not None and front_distance <= 50:
        print(f"전방 거리: {front_distance} cm - 멈춤")
        for motor in motors:
            motor.stop()
	    print("전방 장애물")
			
def main():
    """메인 실행 함수"""
    set_angle(90)  # 전방 각도로 설정
	check_front_and_stop()
	time.sleep(0.05)  # 메인 루프 주기 설정
	
if __name__ == "__main__":
    try:
        initialize_sensor()
        sensor_thread = threading.Thread(target=sensor_read_thread)
        sensor_thread.daemon = True
        sensor_thread.start()
        main()  # 메인 함수 실행
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        pwm.stop()
        GPIO.cleanup()
if __name__ == "__main__":
    main()
