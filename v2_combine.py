import cv2
import numpy as np
import time
import math
import RPi.GPIO as GPIO
import threading
import os
import sys

sys.path.append('/home/dodo/YDLidar-SDK/build/python')  # Adjust the path if necessary
import ydlidar

# PID constants
Kp = 0.50
Ki = 0.00
Kd = 0.04

# PID variables
prev_error = 0.0
integral = 0.0

# Shared variables for LiDAR data
front_distance = float('inf')
left_distance = float('inf')
right_distance = float('inf')

# Lock for synchronizing access to shared variables
lidar_lock = threading.Lock()

# GPIO 경고 메시지 비활성화
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

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
        speed = max(min(speed, 50), -50)
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

# 이미지 처리 함수
def process_image(frame):
    height, width = frame.shape[:2]
    roi = frame[int(height*0.5):height, 0:width]

    hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
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
            diff = line_center_x - (width // 2)
            found = True
        else:
            line_center_x = x_positions[0]
            diff = line_center_x - (width // 2)
            found = True

    if not found:
        line_center_x = width // 2
        diff = 0
        print("선을 감지하지 못했습니다.")

    return line_center_x, diff

# LiDAR thread function
def lidar_thread():
    global front_distance, left_distance, right_distance
    ydlidar.os_init()  # SDK initialization
    laser = ydlidar.CYdLidar()
    ports = ydlidar.lidarPortList()

    # Port settings
    port = "/dev/ttyUSB0"
    for key, value in ports.items():
        port = value

    # LiDAR settings
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 7.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3000)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    # Initialize LiDAR
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan_data = ydlidar.LaserScan()

        while ret and ydlidar.os_isOk() and laser.doProcessSimple(scan_data):
            with lidar_lock:
                # Reset distances for each scan
                front_distance = float('inf')
                left_distance = float('inf')
                right_distance = float('inf')
                time.sleep(0.01)  # CPU 사용량 조절을 위한 딜레이

                for point in scan_data.points:
                    degree_angle = math.degrees(point.angle)
                    if 0.01 <= point.range <= 8.0:
                        # Front (0 degrees)
                        if -1 <= degree_angle <= 1:
                            front_distance = min(front_distance, point.range)
                        # Right (90 degrees)
                        elif 89 <= degree_angle <= 91:
                            right_distance = min(right_distance, point.range)
                        # Left (-90 degrees)
                        elif -91 <= degree_angle <= -89:
                            left_distance = min(left_distance, point.range)
            # Short sleep to prevent high CPU usage
            time.sleep(0.01)
        laser.turnOff()
    laser.disconnecting()

# 장애물 감지 후 라인 복귀 함수
def avoid_obstacle_and_return():
    # LiDAR 데이터 확인
    with lidar_lock:
        ld = left_distance
        rd = right_distance
    print("정지")
    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()
    time.sleep(1)
    print(f"ld : {ld}, rd : {rd}")
    
    # 더 넓은 쪽으로 회피
    if ld > rd:
        print("좌측 회피")
        # 좌측으로 회피하며 대각선 이동
        control_motors(-50, 50)
        time.sleep(1)  # 회피 시간 설정
        control_motors(40, 40)
        time.sleep(1)
        control_motors(50,-50)
        time.sleep(1)
        control_motors(40, 40)
        time.sleep(1)
    else:
        print("우측 회피")
        # 우측으로 회피하며 대각선 이동
        control_motors(50, -50)
        time.sleep(1)  # 회피 시간 설정
        control_motors(40, 40)
        time.sleep(1)
        control_motors(-50,50)
        time.sleep(1)
        control_motors(40, 40)
        time.sleep(1)
    
    time.sleep(1)  # 라인 복귀 전 직진 시간 설정

    # 라인 탐색
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Cannot receive frame.")
            break

        line_center_x, diff = process_image(frame)
        if -60 <= diff <= 60:  # 라인을 찾았을 경우
            print("라인 발견, 라인 추종 모드 복귀")
            break

        # 라인을 찾을 때까지 계속 직진
        control_motors(40, 40)
        time.sleep(0.1)

    # 라인 추종 모드로 복귀
    return

# 장애물 감지 임계값 (단위: 미터)
OBSTACLE_THRESHOLD = 0.6  # 50cm

# 메인 제어 루프
def main():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 424)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("Cannot open camera.")
        return

    prev_time = time.time()

    # LiDAR 스레드 시작
    lidar_thread_instance = threading.Thread(target=lidar_thread)
    lidar_thread_instance.daemon = True
    lidar_thread_instance.start()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Cannot receive frame.")
                break

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time

            # LiDAR 데이터 접근
            with lidar_lock:
                fd = front_distance

            # 장애물 감지 여부 판단
            obstacle_detected = fd < OBSTACLE_THRESHOLD

            if obstacle_detected:

                print("장애물 감지, 회피 동작 시작")
                avoid_obstacle_and_return()
                continue  # 회피 후 라인 복귀 후 루프 재시작

            # 라인 추종 모드
            line_center_x, diff = process_image(frame)

            if math.isnan(line_center_x) or math.isnan(diff):
                print("Warning: Computed value is NaN.")
                continue

            # PID 제어 적용
            pid_value = pid_control(diff, dt)
            base_speed = 40  # 기본 속도 설정

            # 모터 속도 계산
            left_motor_speed = base_speed + pid_value
            right_motor_speed = base_speed - pid_value

            print(f"Left: {left_motor_speed}, Right: {right_motor_speed}")

            # 모터 제어 함수 호출
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
        ydlidar.os_shutdown()  # LiDAR 스레드 종료 신호

    cap.release()
    cv2.destroyAllWindows()

# 프로그램 실행
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    main()

