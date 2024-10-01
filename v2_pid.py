import cv2
import numpy as np
import time

# PID 상수 (조정이 필요할 수 있음)
Kp = 1.0
Ki = 0.0
Kd = 0.0

# PID 제어 변수
prev_error = 0.0
integral = 0.0

# PID 제어 함수
def pid_control(error, dt):
    global prev_error, integral

    proportional = error
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0  # Prevent division by zero
    prev_error = error

    # Return the PID control result
    return Kp * proportional + Ki * integral + Kd * derivative

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

    return diff

# 메인 제어 루프
def main():
    # 카메라 설정
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 424)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 가져올 수 없습니다.")
            break

        # 현재 시간 계산
        current_time = time.time()

        # 이미지 처리 및 중앙값 계산
        diff = process_image(frame)

        if line_center_x is not None:
            # PID 제어를 위한 시간 간격 계산
            dt = time.time() - current_time

            # PID 제어 값 계산
            pid_value = pid_control(diff, dt)

            # 속도 계산
            base_speed = 50  # 기본 속도
            left_motor_speed = base_speed - pid_value  # 왼쪽 속도 제어
            right_motor_speed = base_speed + pid_value  # 오른쪽 속도 제어

            # 모터 속도 출력 (실제 모터 제어 함수 대신 print로 출력)
            print(f"Left Motor Speed: {left_motor_speed}")
            print(f"Right Motor Speed: {right_motor_speed}")

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 카메라 해제
    cap.release()
    cv2.destroyAllWindows()

# 프로그램 실행
if __name__ == "__main__":
    main()
