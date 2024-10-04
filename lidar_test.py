import serial
import time

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

while True:
    distance = read_distance()
    if distance:
        print(f"Distance: {distance} cm")
    else:
        print("Error reading from sensor")
    time.sleep(1)
