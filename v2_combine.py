import cv2
import numpy as np
import time
import math
import RPi.GPIO as GPIO
import sys
import multiprocessing

sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar

# PID 상수
Kp = 0.22
Ki = 0.00
Kd = 0.04

# PID 제어 변수
prev_error = 0.0
integral = 0.0

# OpenCV 최적화 설정
cv2.setUseOptimized(True)

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

# Lidar 초기화 함수
def init_lidar():
    ydlidar.os_init()  # SDK 초기화
    laser = ydlidar.CYdLidar()
    ports = ydlidar.lidarPortList()

    # 포트 설정
    port = "/dev/ttyUSB0"
    for key, value in ports.items():
        port = value

    # Lidar 설정
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 7.0)  # 권장 주파수로 설정
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3000)  # 샘플레이트 유지
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    # Lidar 초기화
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan_data = ydlidar.LaserScan()
        return laser, scan_data
    else:
        print("Lidar 초기화에 실패했습니다.")
        return None, None

# Lidar 데이터 수집 함수
def get_lidar_data(lidar_queue):
    laser, scan_data = init_lidar()
    if laser is None:
        print("라이다 초기화 실패")
        return

    try:
        while True:
            r = laser.doProcessSimple(scan_data)
            front_distance = float('inf')  # 정면 거리
            left_distance = float('inf')   # 좌측 거리
            right_distance = float('inf')  # 우측 거리

            if r:
                for point in scan_data.points:
                    degree_angle = math.degrees(point.angle)
                    if 0.01 <= point.range <= 8.0:

                        # 정면 (0도)
                        if -15 <= degree_angle <= 15:
                            front_distance = min(front_distance, point.range)

                        # 우측 (90도)
                        if 75 <= degree_angle <= 105:
                            right_distance = min(right_distance, point.range)

                        # 좌측 (-90도)
                        if -105 <= degree_angle <= -75:
                            left_distance = min(left_distance, point.range)

            distances = {'front': front_distance, 'left': left_distance, 'right': right_distance}
            if not lidar_queue.full():
                lidar_queue.put(distances)

            time.sleep(0.05)
    finally:
        laser.turnOff()
        laser.disconnecting()

# 이미지 처리 함수 (멀티프로세싱으로 처리)
def process_image(frame_queue, result_queue):
    while True:
        frame = frame_queue.get()
        if frame is None:
            break
        
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

        result_queue.put((line_center_x, diff))

# 메인 제어 루프
def main():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 424)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    prev_time = time.time()

    # 라이다 데이터를 저장할 큐 (최대 크기 설정으로 메모리 사용 제한)
    lidar_queue = multiprocessing.Queue(maxsize=5)
    # 프레임과 결과 저장을 위한 큐
    frame_queue = multiprocessing.Queue(maxsize=1)
    result_queue = multiprocessing.Queue(maxsize=1)

    # 라이다 및 이미지 처리 프로세스 시작
    lidar_process = multiprocessing.Process(target=get_lidar_data, args=(lidar_queue,))
    lidar_process.start()

    image_process = multiprocessing.Process(target=process_image, args=(frame_queue, result_queue))
    image_process.start()

    frame_skip_counter = 0  # 프레임 스킵 카운터

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 가져올 수 없습니다.")
                break

            # 3 프레임당 하나의 이미지 처리
            frame_skip_counter += 1
            if frame_skip_counter % 3 == 0:
                if not frame_queue.full():
                    frame_queue.put(frame)

            # 이미지 처리 결과 가져오기
            if not result_queue.empty():
                line_center_x, diff = result_queue.get()

                if math.isnan(line_center_x) or math.isnan(diff):
                    print("경고: 계산된 값이 NaN입니다.")
                    continue
                current_time = time.time()
                dt = current_time - prev_time
                prev_time = current_time

                # 라이다 데이터 가져오기
                if not lidar_queue.empty():
                    distances = lidar_queue.get()
                    front_dist = distances['front']
                    left_dist = distances['left']
                    right_dist = distances['right']
                    print(f"정면 거리: {front_dist:.2f} m, 좌측 거리: {left_dist:.2f} m, 우측 거리: {right_dist:.2f} m")
                else:
                    front_dist = float('inf')
                    left_dist = float('inf')
                    right_dist = float('inf')

                # 속도 계산
                base_speed = 100
                pid_value = pid_control(diff, dt)

                left_motor_speed = base_speed + pid_value
                right_motor_speed = base_speed - pid_value
                print(f"left : {left_motor_speed} , right : {right_motor_speed}")

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

        # 프로세스 종료
        lidar_process.terminate()
        lidar_process.join()

        frame_queue.put(None)  # 이미지 처리 프로세스 종료 신호
        image_process.terminate()
        image_process.join()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    main()
