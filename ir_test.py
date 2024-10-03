import RPi.GPIO as GPIO
import time

# GPIO 핀 설정
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# GPIO 핀 번호 설정
RIGHT_SENSOR_PIN = 21  # 우측 IR 센서 GPIO 핀
FRONT_SENSOR_PIN = 20  # 앞쪽 IR 센서 GPIO 핀

# 핀을 입력 모드로 설정
GPIO.setup(RIGHT_SENSOR_PIN, GPIO.IN)
GPIO.setup(FRONT_SENSOR_PIN, GPIO.IN)

try:
    while True:
        # 오른쪽 IR 센서 값 읽기
        right_sensor_value = GPIO.input(RIGHT_SENSOR_PIN)
        # 앞쪽 IR 센서 값 읽기
        front_sensor_value = GPIO.input(FRONT_SENSOR_PIN)

        # 센서 값 출력
        print(f"오른쪽 센서 값: {right_sensor_value}, 앞쪽 센서 값: {front_sensor_value}")
        
        if right_sensor_value == 0:
            print("오른쪽에 벽이 있습니다.")
        else:
            print("오른쪽에 벽이 없습니다.")
        
        if front_sensor_value == 0:
            print("앞쪽에 벽이 있습니다.")
        else:
            print("앞쪽에 벽이 없습니다.")
        
        time.sleep(0.5)  # 0.5초 간격으로 센서 값 측정

except KeyboardInterrupt:
    GPIO.cleanup()  # 프로그램 종료 시 GPIO 핀 초기화
