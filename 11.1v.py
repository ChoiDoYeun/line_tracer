import cv2
import numpy as np
import time
import math
import RPi.GPIO as GPIO

# PID 상수
Kp = 5.00
Ki = 0.00
Kd = 0.13

# PID 제어 변수
prev_error = 0.0
integral = 0.0

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
        # 속도를 -100에서 100 사이로 제한
        speed = max(min(speed, 100), -100)
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
    derivative = (error - prev_error) / dt if dt > 0 else 0  # Prevent division by zero
    prev_error = error

    return Kp * proportional + Ki * integral + Kd * derivative

# 가속도를 기반으로 속도 변화를 부드럽게 적용하는 함수
def accelerate_to_target_speed(target_speed, current_speed, max_acceleration=5):
    if abs(target_speed - current_speed) > max_acceleration:
        if target_speed > current_speed:
            current_speed += max_acceleration
        else:
            current_speed -= max_acceleration
    else:
        current_speed = target_speed
    return current_speed

# 모터 제어 함수 (가속도 적용)
def control_motors(left_speed, right_speed, prev_left_speed, prev_right_speed):
    # 속도 범위 제한 (리밋)
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)

    # 가속도 기반 속도 변화
    left_speed = accelerate_to_target_speed(left_speed, prev_left_speed)
    right_speed = accelerate_to_target_speed(right_speed, prev_right_speed)

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
    
    return left_speed, right_speed  # 업데이트된 속도를 반환

# 이미지 처리 함수
def process_image(frame):
    # 이미지를 HLS로 변환하고 S 채널 추출
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    s_channel = hls[:, :, 2]

    # Gaussian 블러 적용
    blurred = cv2.GaussianBlur(s_channel, (5, 5), 0)

    # Canny 에지 감지 적용
    canny_edges = cv2.Canny(blurred, 50, 150)

    # Hough Line Transform 적용
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
            diff = line_center_x - 211  # 기준점 211
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

# 메인 제어 루프
def main():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 424)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    prev_time = time.time()
    prev_left_motor_speed = 0
    prev_right_motor_speed = 0

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

            print(f"diff : {diff}")
            if -30 <= diff <= 30:
                pid_value = 0 
            else:
                pid_value = pid_control(diff, dt)

            base_speed = 30
            left_motor_speed = base_speed + pid_value
            right_motor_speed = base_speed - pid_value

            print(f"left : {left_motor_speed} , right : {right_motor_speed}")

            prev_left_motor_speed, prev_right_motor_speed = control_motors(left_motor_speed, right_motor_speed, prev_left_motor_speed, prev_right_motor_speed)

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

    cap.release()
    cv2.destroyAllWindows()

# 프로그램 실행
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    main()
