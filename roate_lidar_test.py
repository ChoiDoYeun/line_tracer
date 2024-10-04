import RPi.GPIO as GPIO
import time
import serial

# GPIO 핀 설정
servo_pin = 4  # 서보모터를 연결할 GPIO 핀 번호

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM 주파수 설정 (서보모터의 주파수는 보통 50Hz)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    # 각도를 서보모터의 듀티사이클로 변환 (0도에서 180도 사이)
    duty = 2 + (angle / 18)
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.1)  # 서보 모터가 이동할 시간을 줌
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

# 시리얼 포트 설정 (라즈베리파이의 기본 UART 포트는 /dev/serial0)
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)

def read_distance():
    # 센서에서 데이터를 읽어들임
    time.sleep(0.01)
    ser.write(b'\x42\x57\x02\x00\x00\x00\x01\x06')  # 데이터 요청 명령
    response = ser.read(9)  # 응답 데이터 읽기 (9바이트)

    if len(response) == 9:
        # 거리 값 추출 (바이트 데이터에서 거리 정보 추출)
        distance = response[2] + (response[3] << 8)
        return distance
    else:
        return None

try:
    # 90도 (중립)
    set_angle(90)
    print("중립 (90도)에서 거리 측정 중...")
    time.sleep(0.01)  # 모터가 안정될 시간을 줌
    distance = read_distance()
    if distance:
        print(f"중립 위치 거리: {distance} cm")
    else:
        print("거리 측정 오류")

    # 0도 (우측)
    set_angle(0)
    print("우측 (0도)에서 거리 측정 중...")
    time.sleep(0.01)
    distance = read_distance()
    if distance:
        print(f"우측 위치 거리: {distance} cm")
    else:
        print("거리 측정 오류")

    # 180도 (좌측)
    set_angle(180)
    print("좌측 (180도)에서 거리 측정 중...")
    time.sleep(0.01)
    distance = read_distance()
    if distance:
        print(f"좌측 위치 거리: {distance} cm")
    else:
        print("거리 측정 오류")

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
