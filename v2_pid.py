import cv2
import numpy as np
import time
import math
import RPi.GPIO as GPIO

# PID 상수 (적절하게 조정해야 함)
Kp = 8.00
Ki = 0.00
Kd = 0.01

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

# 왼쪽 모터와 오른쪽 모터 설정
motor1 = MotorController(18, 17, 27)  # 왼쪽 모터
motor2 = MotorController(16, 13, 26)  # 오른쪽 모터

# PID 제어 함수
def pid_control(error, dt):
    global prev_error, integral

    proportional = error
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0  # Prevent division by zero
    prev_error = error

    # Return the PID control result
    return Kp * proportional + Ki * integral + Kd * derivative

# 모터 제어 함수 (보정 적용)
def control_motors(left_speed, right_speed):
    # 우측 모터 속도에 보정 적용 (1.1853배)
    right_speed = right_speed * 1.1853

    # 속도 범위 제한 (리밋)
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)

    if left_speed >= 0:
        motor1.forward(left_speed)
    else:
        motor1.backward(-left_speed)

    if right_speed >= 0:
        motor2.forward(right_speed)
    else:
        motor2.backward(-right_speed)

# 이미지 처리 함수
def process_image(frame):
    # 이미지를 HLS로 변환
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    # S 채널 추출
    s_channel = hls[:, :, 2]

    # Gaussian 블러 적용
    blurred = cv2.GaussianBlur(s_channel, (5, 5), 0)

    # Canny 에지 감지 적용
    canny_edges = cv2.Canny(blurred, 50, 150)

    # Hough Line Transform 적용
    lines = cv2.HoughLinesP(canny_edges, 1, np.pi / 180, threshold=50, minLineLength=10, maxLineGap=10)

    # 선의 중앙값 계산
    line_center_x, diff = None, None
    found = False
    for y in range(240, 119, -5):  # y=240부터 y=120까지 5 단위로 올라감
        x_positions = []
        if lines is not None:
            # 각 라인에서 해당 y 좌표에 대한 x 값을 찾음
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if y1 <= y <= y2 or y2 <= y <= y1:
                    x = int(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
                    x_positions.append(x)

            # 두 선이 감지되었다면, 두 선 사이의 중앙값을 계산
            if len(x_positions) == 2:
                left_x, right_x = sorted(x_positions)
                line_center_x = (left_x + right_x) // 2
                diff = line_center_x - 211  # 기준점 211

                found = True
                break  # 선을 찾으면 반복 종료

    # 라인이 감지되지 않았을 때 기본값 설정
    if not found:
        line_center_x = 211  # 중앙으로 설정
        diff = 0

    return line_center_x, diff

# 메인 제어 루프
def main():
    # 카메라 설정
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 424)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 가져올 수 없습니다.")
                break

            # 현재 시간 계산
            current_time = time.time()

            # 이미지 처리 및 중앙값 계산
            line_center_x, diff = process_image(frame)

            # NaN 검사 추가
            if math.isnan(line_center_x) or math.isnan(diff):
                print("경고: 계산된 값이 NaN입니다.")
                continue

            # PID 제어를 위한 시간 간격 계산
            dt = time.time() - current_time

            # PID 제어 값 계산
            pid_value = pid_control(diff, dt)

            # 속도 계산
            base_speed = 70  # 기본 속도
            left_motor_speed = base_speed - pid_value  # 왼쪽 속도 제어
            right_motor_speed = base_speed + pid_value  # 오른쪽 속도 제어

            print(f"left : {left_motor_speed} , right : {right_motor_speed}")

            # 모터 제어 함수 호출
            control_motors(left_motor_speed, right_motor_speed)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 종료 시 모든 모터 정지 및 GPIO 정리
        motor1.stop()
        motor2.stop()
        motor1.cleanup()
        motor2.cleanup()

    # 카메라 해제
    cap.release()
    cv2.destroyAllWindows()

# 프로그램 실행
if __name__ == "__main__":
    # GPIO 모드 설정
    GPIO.setmode(GPIO.BCM)
    main()
