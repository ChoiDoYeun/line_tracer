import cv2
import numpy as np
import time
import math
import RPi.GPIO as GPIO
import sys
sys.path.append('/home/dodo/YDLidar-SDK/build/python')
import ydlidar

# PID 상수
Kp = 0.22
Ki = 0.00
Kd = 0.04

# PID 제어 변수
prev_error = 0.0
integral = 0.0

# MotorController 클래스 (생략)

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
def get_lidar_data(scan_data, laser):
    r = laser.doProcessSimple(scan_data)
    front_distance = float('inf')  # 정면 거리
    left_distance = float('inf')   # 좌측 거리
    right_distance = float('inf')  # 우측 거리

    if r:
        for point in scan_data.points:
            degree_angle = math.degrees(point.angle)  # 각도를 도 단위로 변환
            if 0.01 <= point.range <= 8.0:  # 유효한 거리 데이터(0.01m ~ 8m)

                # 정면 (0도)
                if -15 <= degree_angle <= 15:  # 0도 근처
                    front_distance = min(front_distance, point.range)

                # 우측 (90도)
                if 75 <= degree_angle <= 105:  # 90도 근처 (우측)
                    right_distance = min(right_distance, point.range)

                # 좌측 (-90도)
                if -105 <= degree_angle <= -75:  # -90도 근처 (좌측)
                    left_distance = min(left_distance, point.range)

    return front_distance, left_distance, right_distance

# 이미지 처리 함수 (생략)

# 메인 제어 루프
def main():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 424)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    prev_time = time.time()

    # Lidar 초기화
    laser, scan_data = init_lidar()
    if laser is None:
        return

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

            # 라이다 데이터를 읽어옴
            front_dist, left_dist, right_dist = get_lidar_data(scan_data, laser)
            print(f"정면 거리: {front_dist:.2f} m, 좌측 거리: {left_dist:.2f} m, 우측 거리: {right_dist:.2f} m")

            # 라이다 거리 기반으로 장애물 회피 로직 추가 가능
            if front_dist < 0.5:  # 정면에 50cm 이내 장애물이 있는 경우
                print("장애물 감지! 속도를 줄입니다.")
                base_speed = 20  # 속도 감소
            else:
                base_speed = 100  # 정상 속도 유지

            pid_value = pid_control(diff, dt)

            # 속도 계산
            left_motor_speed = base_speed + pid_value
            right_motor_speed = base_speed - pid_value

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
        laser.turnOff()  # Lidar 종료
        laser.disconnecting()

    cap.release()
    cv2.destroyAllWindows()

# 프로그램 실행
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    main()
