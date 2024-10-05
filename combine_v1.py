import cv2
import numpy as np
import time
import math
import RPi.GPIO as GPIO
import sys
import threading
import ydlidar
import traceback

# PID 상수
Kp = 0.22
Ki = 0.00
Kd = 0.04

# PID 제어 변수
prev_error = 0.0
integral = 0.0

# 장애물 감지 변수
obstacle_detected = False
left_distance = float('inf')
right_distance = float('inf')
obstacle_lock = threading.Lock()

# 라이다 데이터 갱신 이벤트
lidar_data_event = threading.Event()

# MotorController 클래스
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

GPIO.setmode(GPIO.BCM)

# 모터 초기화
motor1 = MotorController(18, 17, 27)  # left front
motor2 = MotorController(22, 23, 24)  # right front
motor3 = MotorController(9, 10, 11)   # left back
motor4 = MotorController(25, 8, 7)    # right back

# PID 제어 함수
def pid_control(error, dt):
    global prev_error, integral
    
    proportional = error
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0
    prev_error = error

    return Kp * proportional + Ki * integral + Kd * derivative

# 모터 제어 함수
def control_motors(left_speed, right_speed):
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)

    if left_speed >= 0:
        motor1.forward(left_speed)
        motor3.forward(left_speed)
    else:
        motor1.backward(-left_speed)
        motor3.backward(-left_speed)

    if right_speed >= 0:
        motor2.forward(right_speed)
        motor4.forward(right_speed)
    else:
        motor2.backward(-right_speed)
        motor4.backward(-right_speed)

# 이미지 처리 함수 (선 감지)
def process_image(frame):
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    s_channel = hls[:, :, 2]
    blurred = cv2.GaussianBlur(s_channel, (5, 5), 0)
    canny_edges = cv2.Canny(blurred, 50, 150)
    lines = cv2.HoughLinesP(canny_edges, 1, np.pi / 180, threshold=20, minLineLength=5, maxLineGap=10)

    line_center_x, diff = None, None
    found = False

    if lines is not None:
        x_positions = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            x_mid = (x1 + x2) // 2
            x_positions.append(x_mid)

        x_positions.sort()
        num_positions = len(x_positions)

        if num_positions >= 2:
            left_x = x_positions[0]
            right_x = x_positions[-1]
            line_center_x = (left_x + right_x) // 2
            diff = line_center_x - 211
            found = True
        else:
            line_center_x = x_positions[0]
            diff = line_center_x - 211
            found = True

    if not found:
        line_center_x = 211
        diff = 0
        print("선을 감지하지 못했습니다.")

    return line_center_x, diff

# 라이다 데이터 처리 스레드 함수
def lidar_thread(laser):
    global obstacle_detected, left_distance, right_distance
    scan_data = ydlidar.LaserScan()
    while True:
        try:
            r = laser.doProcessSimple(scan_data)
            if r:
                front_distance = float('inf')
                left_distance_temp = float('inf')
                right_distance_temp = float('inf')
                for point in scan_data.points:
                    degree_angle = math.degrees(point.angle)
                    if 0.01 <= point.range <= 8.0:
                        # 정면 감지
                        if -5 <= degree_angle <= 5:
                            front_distance = min(front_distance, point.range)
                        # 좌측 감지 (90도)
                        if 85 <= degree_angle <= 95:
                            left_distance_temp = min(left_distance_temp, point.range)
                        # 우측 감지 (-90도)
                        if -95 <= degree_angle <= -85:
                            right_distance_temp = min(right_distance_temp, point.range)
                with obstacle_lock:
                    obstacle_detected = front_distance <= 0.8  # 80cm 이내 장애물 감지
                    left_distance = left_distance_temp
                    right_distance = right_distance_temp
                    lidar_data_event.set()  # 라이다 데이터 갱신 이벤트 설정
            else:
                print("라이다 데이터 수집 실패. 라이다 재초기화 중...")
                # 라이다 재초기화 코드
                laser.turnOff()
                laser.disconnecting()
                time.sleep(1)  # 잠시 대기 후 다시 시도
                laser.initialize()
                laser.turnOn()
                print("라이다 재초기화 완료")
            time.sleep(0.01)  # 주기 조정
        except Exception as e:
            print("라이다 스레드 예외 발생:", e)
            traceback.print_exc()
            time.sleep(1)  # 예외 발생 시 잠시 대기

# 장애물 회피 동작 함수
def avoid_obstacle():
    global left_distance, right_distance  # 전역 변수로 선언

    # 최초로 좌우측 거리를 비교하여 회피 방향 결정
    if left_distance > right_distance:
        print("좌측으로 회피")
        control_motors(-50, 50)  # 좌측으로 회피
        time.sleep(0.75)  # 일정 시간 이동
        print("좌측 최초 회피 끝")
    else:
        print("우측으로 회피")
        control_motors(50, -50)  # 우측으로 회피
        time.sleep(0.75)  # 일정 시간 이동
        print("우측 최초 회피 끝")

    # 회피 후 모터 멈춤
    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()

    time.sleep(0.1)  # 정지 후 잠시 대기

    # 라이다 데이터 갱신 대기
    print("라이다 데이터 갱신 대기 중...")
    if not lidar_data_event.wait(timeout=2.0):  # 최대 2초 대기
        print("라이다 데이터 갱신 시간 초과")
        return  # 데이터 갱신이 안되면 함수 종료
    lidar_data_event.clear()  # 이벤트 초기화

    with obstacle_lock:
        print(f"갱신된 좌측 거리: {left_distance} m, 갱신된 우측 거리: {right_distance} m")

    # 갱신된 거리 데이터를 기반으로 추가 회피 동작
    if left_distance > right_distance:
        print("계속 좌측으로 회피")
        control_motors(-50, 50)  # 좌측 회피 계속
        time.sleep(0.25)  # 일정 시간 이동
        print("좌측 재 회피 끝")
    else:
        print("우측으로 전환")
        control_motors(50, -50)  # 우측 회피로 전환
        time.sleep(0.25)  # 일정 시간 이동
        print("우측 재 회피 끝")

    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()
    time.sleep(0.25)  # 회피 후 잠시 대기

# 메인 제어 루프
def main():
    global laser  # laser 변수를 전역 변수로 선언

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 424)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    # Lidar 설정
    ydlidar.os_init()
    laser = ydlidar.CYdLidar()  # 전역 변수 laser 초기화
    ports = ydlidar.lidarPortList()
    port = "/dev/ttyUSB0"
    for key, value in ports.items():
        port = value
    # 기존 라이다 설정 유지
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 7.0)  # 권장 주파수로 설정
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3000)  # 샘플레이트 유지
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
    ret = laser.initialize()

    if ret:
        ret = laser.turnOn()
        if not ret:
            print("라이다를 켤 수 없습니다.")
            return
        # 라이다 스레드 시작
        lidar_thread_instance = threading.Thread(target=lidar_thread, args=(laser,), daemon=True)
        lidar_thread_instance.start()
    else:
        print("라이다를 초기화할 수 없습니다.")
        return

    prev_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 가져올 수 없습니다.")
                break

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time

            line_center_x, diff = process_image(frame)

            if math.isnan(line_center_x) or math.isnan(diff):
                print("경고: 계산된 값이 NaN입니다.")
                continue

            if -60 <= diff <= 60:
                base_speed = 100
                pid_value = 0
            else:
                base_speed = 20
                pid_value = pid_control(diff, dt)

            left_motor_speed = base_speed + pid_value
            right_motor_speed = base_speed - pid_value

            # 장애물 감지 확인
            with obstacle_lock:
                if obstacle_detected:
                    print("장애물 감지, 멈춤")
                    motor1.stop()
                    motor2.stop()
                    motor3.stop()
                    motor4.stop()
                    time.sleep(0.5)

                    # 회피 동작 수행
                    avoid_obstacle()
                    continue  # 회피 동작 후 다음 루프로 이동

            control_motors(left_motor_speed, right_motor_speed)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()
        motor1.cleanup()
        motor2.cleanup()
        motor3.cleanup()
        motor4.cleanup()
        laser.turnOff()
        laser.disconnecting()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    main()
