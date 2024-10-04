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
    # 이미지를 HLS로 변환하고 S 채널 추출
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    s_channel = hls[:, :, 2]

    # Gaussian 블러 적용
    blurred = cv2.GaussianBlur(s_channel, (5, 5), 0)

    # Canny 에지 감지 적용
    edges = cv2.Canny(blurred, 50, 150)

    # 관심 영역(ROI) 설정
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # ROI 다각형 정의 (이미지의 하단 절반)
    polygon = np.array([[
        (0, height),
        (0, height * 0.5),
        (width, height * 0.5),
        (width, height)
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    roi_edges = cv2.bitwise_and(edges, mask)

    # Hough Line Transform 적용
    lines = cv2.HoughLinesP(roi_edges, 1, np.pi / 180, threshold=20, minLineLength=20, maxLineGap=300)

    # 선 분류 및 평균 계산
    left_fit = []
    right_fit = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 수직선은 제외
            if x1 == x2:
                continue
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            # 기울기에 따라 좌우 선 분류
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))

    # 평균 선 계산
    left_line = make_coordinates(frame, np.average(left_fit, axis=0)) if left_fit else None
    right_line = make_coordinates(frame, np.average(right_fit, axis=0)) if right_fit else None

    # 중앙선 계산
    if left_line is not None and right_line is not None:
        left_x_bottom = left_line[0]
        right_x_bottom = right_line[0]
        line_center_x = (left_x_bottom + right_x_bottom) // 2
        diff = line_center_x - (width // 2)
    else:
        line_center_x = width // 2
        diff = 0
        print("선을 충분히 검출하지 못했습니다.")

    return line_center_x, diff

def make_coordinates(frame, line_parameters):
    slope, intercept = line_parameters
    y1 = frame.shape[0]  # 이미지 하단
    y2 = int(y1 * 0.6)   # 이미지의 60% 지점
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [x1, y1, x2, y2]

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

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임을 가져올 수 없습니다.")
                break

            # 현재 시간 계산
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time  # 이전 시간 업데이트

            # 이미지 처리 및 중앙값 계산
            line_center_x, diff = process_image(frame)

            # NaN 검사 추가
            if math.isnan(line_center_x) or math.isnan(diff):
                print("경고: 계산된 값이 NaN입니다.")
                continue

            # PID 제어 값 계산
            print(f"diff : {diff}")
            if -90 <= diff <= 90:
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
