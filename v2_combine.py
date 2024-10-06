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
Kp = 0.22
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

# Main control loop
def main():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 424)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("Cannot open camera.")
        return

    prev_time = time.time()

    # Start LiDAR thread
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

            line_center_x, diff = process_image(frame)

            if math.isnan(line_center_x) or math.isnan(diff):
                print("Warning: Computed value is NaN.")
                continue

            # Within threshold, base_speed = 50; outside threshold, base_speed = 0
            if -60 <= diff <= 60:
                base_speed = 100  # Maintain speed within threshold
                pid_value = 0  # Set correction value to 0
            else:
                base_speed = 20  # Stop when outside threshold
                pid_value = pid_control(diff, dt)

            # Calculate speeds
            left_motor_speed = base_speed + pid_value
            right_motor_speed = base_speed - pid_value

            print(f"Left: {left_motor_speed}, Right: {right_motor_speed}")

            # Access LiDAR data
            with lidar_lock:
                fd = front_distance
                ld = left_distance
                rd = right_distance

            # Optionally, print or use the LiDAR data
            print(f"Front distance: {fd * 100:.2f} cm" if fd != float('inf') else "No object detected in front.")
            print(f"Left distance: {ld * 100:.2f} cm" if ld != float('inf') else "No object detected on the left.")
            print(f"Right distance: {rd * 100:.2f} cm" if rd != float('inf') else "No object detected on the right.")

            # Call motor control function
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
        ydlidar.os_shutdown()  # Signal LiDAR thread to stop

    cap.release()
    cv2.destroyAllWindows()

# Program execution
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    main()
