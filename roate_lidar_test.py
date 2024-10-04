import RPi.GPIO as GPIO
import time
import serial

# GPIO 핀 설정
servo_pin = 4  # 서보모터를 연결할 GPIO 핀 번호

# 시리얼 포트 설정 (라즈베리파이의 기본 UART 포트는 /dev/serial0)
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.1)

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM 주파수 설정 (서보모터의 주파수는 보통 50Hz)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    """서보모터의 각도를 설정"""
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)  # 각도 설정 후 잠시 대기

def read_distance():
    """센서에서 거리 데이터를 읽어옴"""
    ser.reset_input_buffer()  # 시리얼 버퍼 초기화
    ser.write(b'\x42\x57\x02\x00\x00\x00\x01\x06')  # 데이터 요청 명령
    time.sleep(0.2)
    response = ser.read(9)  # 응답 데이터 읽기 (9바이트)
    
    if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
        distance = response[2] + response[3] * 256
        return distance
    else:
        return None

try:
    # 우측(0도), 중립(90도), 좌측(180도) 각도로 이동하면서 거리 측정
    angles = [0, 90, 180]
    positions = ["우측", "중앙", "좌측"]

    for i in range(3):
        set_angle(angles[i])
        print(f"{positions[i]}로 회전")
        time.sleep(0.3)  # 서보모터가 이동할 시간을 최소화

        # 거리 데이터 읽기
        distance = read_distance()
        if distance:
            print(f"{positions[i]} 거리: {distance} cm")
        else:
            print(f"{positions[i]}에서 센서 오류 발생")

except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    pwm.stop()
    GPIO.cleanup()
