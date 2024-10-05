import os
import sys
import math
import RPi.GPIO as GPIO
import time
import threading
import queue

sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar

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
        speed = max(min(speed, 60), -60)  # 속도 제한
        self.pwm.ChangeDutyCycle(abs(speed))
        if speed > 0:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)

    def stop(self):
        self.set_speed(0)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup([self.en, self.in1, self.in2])

# Lidar 데이터를 수집하는 스레드 함수
def lidar_thread_func(scan_queue, laser):
    while True:
        scan_data = ydlidar.LaserScan()
        r = laser.doProcessSimple(scan_data)
        if r:
            scan_queue.put(scan_data)
        else:
            print("Failed to get Lidar Data.")
        time.sleep(0.05)  # Lidar 스캔 주기에 맞게 조정

# 필요한 각도 범위만 필터링하여 거리 계산
def process_scan_data(scan_data):
    front_distance = float('inf')  # 정면 거리
    right_distance = float('inf')  # 우측 거리

    for point in scan_data.points:
        if 0.01 <= point.range <= 8.0:  # 유효한 거리 데이터
            degree_angle = math.degrees(point.angle)

            # 정면 (0도)
            if -5 <= degree_angle <= 5:
                front_distance = min(front_distance, point.range)

            # 우측 (90도)
            if 85 <= degree_angle <= 95:
                right_distance = min(right_distance, point.range)

    # 디버깅 정보 출력
    print(f"정면 거리: {front_distance * 100:.2f} cm, 우측 거리: {right_distance * 100:.2f} cm")

    return front_distance, right_distance

# PID 컨트롤러 클래스
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.15):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint  # 목표 거리 (예: 15cm)
        self.previous_error = 0
        self.integral = 0

    def compute(self, measurement):
        error = self.setpoint - measurement
        self.integral += error * 0.05  # 적분 시간 간격 (여기서는 0.05초)
        derivative = (error - self.previous_error) / 0.05
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

if __name__ == "__main__":
    # GPIO 설정
    GPIO.setmode(GPIO.BCM)

    # 모터 초기화
    motor1 = MotorController(18, 17, 27)  # left front
    motor2 = MotorController(22, 23, 24)  # right front
    motor3 = MotorController(9, 10, 11)   # left back
    motor4 = MotorController(25, 8, 7)    # right back

    # PID 컨트롤러 초기화
    pid = PIDController(kp=200, ki=0, kd=50, setpoint=0.15)  # kp, ki, kd 값은 튜닝 필요

    # YDLidar 설정
    ydlidar.os_init()
    laser = ydlidar.CYdLidar()
    ports = ydlidar.lidarPortList()

    port = "/dev/ttyUSB0"
    for key, value in ports.items():
        port = value

    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 7.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3000)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

    # Lidar 초기화
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()

        # 스레드를 위한 큐 생성
        scan_queue = queue.Queue()

        # Lidar 스레드 시작
        lidar_thread = threading.Thread(target=lidar_thread_func, args=(scan_queue, laser))
        lidar_thread.daemon = True
        lidar_thread.start()

        try:
            while True:
                # 최신의 Lidar 데이터 가져오기
                if not scan_queue.empty():
                    scan_data = scan_queue.get()
                    front_dist, right_dist = process_scan_data(scan_data)

                    # 정면에 벽이 가까울 때 우회전
                    if front_dist <= 0.3:  # 정면에 벽이 30cm 이내일 때
                        print("정면에 벽 감지, 우회전 중...")
                        motor1.set_speed(40)
                        motor2.set_speed(-40)
                        motor3.set_speed(40)
                        motor4.set_speed(-40)
                        time.sleep(0.5)
                        continue  # 다음 루프로 이동

                    # PID 제어 계산
                    control_output = pid.compute(right_dist)

                    # 모터 속도 결정
                    base_speed = 40  # 기본 속도

                    # 좌우 모터 속도에 PID 출력 적용
                    left_speed = base_speed + control_output
                    right_speed = base_speed - control_output

                    # 속도 제한 적용
                    left_speed = max(min(left_speed, 60), -60)
                    right_speed = max(min(right_speed, 60), -60)

                    # 모터 구동
                    motor1.set_speed(left_speed)
                    motor2.set_speed(right_speed)
                    motor3.set_speed(left_speed)
                    motor4.set_speed(right_speed)

                    print(f"PID 출력: {control_output:.2f}, 좌측 모터 속도: {left_speed:.2f}, 우측 모터 속도: {right_speed:.2f}")

                else:
                    # Lidar 데이터가 없을 때 잠시 대기
                    time.sleep(0.01)

        except KeyboardInterrupt:
            # 정지 및 정리
            motor1.stop()
            motor2.stop()
            motor3.stop()
            motor4.stop()
            motor1.cleanup()
            motor2.cleanup()
            motor3.cleanup()
            motor4.cleanup()

        # Lidar 종료
        laser.turnOff()
    laser.disconnecting()
