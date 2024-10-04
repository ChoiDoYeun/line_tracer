import cv2
import numpy as np
import time
import math
import RPi.GPIO as GPIO

# PID 상수 (적절하게 조정해야 함)
Kp = 4.80
Ki = 0.00
Kd = 0.05

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
motor1 = MotorController(18, 17, 27) # left front
motor2 = MotorController(22, 23, 24) # right front
motor3 = MotorController(9, 10, 11) # left back
motor4 = MotorController(25, 8, 7) # right back

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
    # 속도 범위 제한 (리밋)
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
    # 이미지를 HLS로 변환
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    # S 채널 추출
    s_channel = hls[:, :, 2]

    # Gaussian 블러 적용
    blurred = cv2.GaussianBlur(s_channel, (5, 5), 0)

    # Canny 에지 감지 적용
    canny_edges = cv2.Canny(blurred, 50, 150)

    # Hough Line Transform 적용
    lines = cv2.HoughLinesP(canny_edges, 1, np.pi / 180, threshold=20, minLineLength=5, maxLineGap=10)

    # 선의 중앙값 계산
    line_center_x, diff = None, None
    found = False
    for y in range(240, 50, -1):  # y=240부터 y=0까지 1 단위로 내려감
        x_positions = []
        if lines is not None:
            # 각 라인에서 해당 y 좌표에 대한 x 값을 찾음
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if y1 <= y <= y2 or y2 <= y <= y1:
                    if y2 - y1 !=0:
                        x = int(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
                        x_positions.append(x)
                    else :
                        x = int((x1+x2)/2)
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
        print("no detect")

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

    prev_time = time.time()  # 이전 시간을 저장
    detection_interval = 0.0083  # 라인 검출 간격 (0.01초, 즉 100FPS 정도로 설정)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 가져올 수 없습니다.")
                break

            # 현재 시간 계산
            current_time = time.time()
            dt = current_time - prev_time

            # 라인 검출은 일정 주기마다만 실행
            if dt >= detection_interval:
                prev_time = current_time  # 이전 시간 업데이트

                # 이미지 처리 및 중앙값 계산
                line_center_x, diff = process_image(frame)

                # NaN 검사 추가
                if math.isnan(line_center_x) or math.isnan(diff):
                    print("경고: 계산된 값이 NaN입니다.")
                    continue

                # PID 제어 값 계산
                print(f"diff : {diff}")
                if -70 <= diff <= 70:
                    pid_value = 0  # error가 -90에서 90 사이일 경우 PID 보정을 하지 않음
                else:
                    pid_value = pid_control(diff, dt)  # error가 범위를 벗어나면 PID 보정 적용

                # 속도 계산
                base_speed = 50  # 기본 속도
                left_motor_speed = base_speed + pid_value  # 왼쪽 속도 제어
                right_motor_speed = base_speed - pid_value  # 오른쪽 속도 제어

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
        motor3.stop()
        motor4.stop()
        motor1.cleanup()
        motor2.cleanup()
        motor3.cleanup()
        motor4.cleanup()

    # 카메라 해제
    cap.release()
    cv2.destroyAllWindows()

# 프로그램 실행
if __name__ == "__main__":
    # GPIO 모드 설정
    GPIO.setmode(GPIO.BCM)
    main()
