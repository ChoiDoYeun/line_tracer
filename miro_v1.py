import RPi.GPIO as GPIO
import time
import serial

# GPIO 핀 설정
servo_pin = 4  # 서보모터를 연결할 GPIO 핀 번호
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.01)  # 통신 속도 설정

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM 주파수 설정 (서보모터의 주파수는 보통 50Hz)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

# MotorController 클래스 정의 (코드는 동일하므로 생략)

# 모터 초기화 (코드는 동일하므로 생략)

# 서보모터 제어 함수
def set_angle(angle):
    """서보모터의 각도를 설정"""
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # 대기 시간을 0.5초로 조정

# 거리 측정 함수
def read_distance():
    """TF Luna 센서에서 거리 데이터를 읽어옴"""
    ser.reset_input_buffer()
    ser.write(b'\x42\x57\x02\x00\x00\x00\x01\x06')  # 데이터 요청 명령
    response = ser.read(9)
    
    if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
        distance = response[2] + response[3] * 256
        return distance
    else:
        return None

# 우회전 로직
def dynamic_right_turn():
    """동적으로 우회전을 수행하는 함수"""
    max_turn_time = 5  # 최대 회전 시간 설정 (예: 5초)
    start_time = time.time()

    set_angle(0)  # 우측 각도 설정
    time.sleep(0.5)  # 서보모터가 움직일 시간을 줌

    while True:
        # 우측 거리 측정
        right_distance = read_distance()

        # 전방 거리 측정
        set_angle(90)  # 전방 각도 설정
        time.sleep(0.5)
        front_distance = read_distance()

        # 디버깅 출력
        if right_distance is not None:
            print(f"우측 거리: {right_distance} cm")
        else:
            print("우측 거리 데이터를 읽지 못했습니다.")

        if front_distance is not None:
            print(f"전방 거리: {front_distance} cm")
        else:
            print("전방 거리 데이터를 읽지 못했습니다.")

        # 회전 조건 판단
        if front_distance is not None and front_distance <= 100:
            if right_distance is not None and right_distance > 30:
                print("우회전 중...")
                # 우회전 모터 동작
                motor1.forward(30)
                motor2.backward(30)
                motor3.forward(30)
                motor4.backward(30)
            else:
                print("우측에 장애물이 있습니다. 회전을 멈춥니다.")
                break
        else:
            print("전방이 열렸습니다. 회전을 멈춥니다.")
            break

        # 최대 회전 시간 체크
        if time.time() - start_time > max_turn_time:
            print("최대 회전 시간을 초과했습니다. 회전을 멈춥니다.")
            break

        time.sleep(0.1)  # 반복 주기 설정

    # 모터 정지
    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()

# 전방 거리 확인 및 멈춤 로직 (코드는 동일하므로 생략)

def main():
    """메인 실행 함수"""
    while True:
        check_front_and_stop()

if __name__ == "__main__":
    try:
        main()  # 메인 함수 실행
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        pwm.stop()
        GPIO.cleanup()
