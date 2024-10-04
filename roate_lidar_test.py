import RPi.GPIO as GPIO
import time
import serial

# GPIO 핀 설정
servo_pin = 4  # 서보모터를 연결할 GPIO 핀 번호

# 시리얼 포트 설정 (라즈베리파이의 기본 UART 포트는 /dev/serial0)
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)

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
    time.sleep(0.5)  # 각도 설정 후 잠시 대기

def read_distance():
    """센서에서 거리 데이터를 읽어옴"""
    ser.flushInput()  # 시리얼 버퍼 초기화
    time.sleep(0.1)
    count = ser.in_waiting  # 수신 버퍼에 있는 바이트 수
    if count >= 9:
        recv = ser.read(9)  # 9바이트 읽기
        ser.reset_input_buffer()  # 입력 버퍼 초기화

        # 데이터 검증 (시작 바이트 확인)
        if recv[0] == 0x59 and recv[1] == 0x59:
            distance = recv[2] + recv[3] * 256
            return distance
    return None

try:
    # 우측(0도), 중립(90도), 좌측(180도) 각도로 이동하면서 거리 측정
    angles = [0, 90, 180]
    positions = ["우측", "중앙", "좌측"]

    for i in range(3):
        set_angle(angles[i])
        print(f"{positions[i]}로 회전")
        time.sleep(1)  # 서보모터가 이동할 시간을 줌

        # 거리 데이터를 여러 번 읽어서 평균값 계산
        distances = []
        for _ in range(5):
            distance = read_distance()
            if distance:
                distances.append(distance)
            time.sleep(0.1)
        
        if distances:
            avg_distance = sum(distances) / len(distances)
            print(f"{positions[i]} 거리: {avg_distance:.2f} cm")
        else:
            print(f"{positions[i]}에서 센서 오류 발생")
        
        time.sleep(1)  # 다음 위치로 이동하기 전 잠시 대기

except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    pwm.stop()
    GPIO.cleanup()
