# 장애물 감지 후 라인 복귀 함수
def avoid_obstacle_and_return():
    # LiDAR 데이터 확인
    with lidar_lock:
        ld = left_distance
        rd = right_distance
    print("정지")
    
    # 먼저 diff를 -60 ~ 60 사이로 맞추기
    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라 프레임을 받을 수 없습니다.")
            break
        
        line_center_x, diff = process_image(frame)
        
        # diff가 -60 ~ 60 사이에 들어올 때까지 회전하며 조정
        if -60 <= diff <= 60:
            print("diff가 적절한 범위에 들어왔습니다. 멈춤을 준비합니다.")
            break  # diff 조정 완료, while 루프 종료
        
        # diff가 -60 미만이면 왼쪽 회전
        if diff < -60:
            control_motors(20, -20)  # 좌측 회전
        # diff가 60 초과면 오른쪽 회전
        elif diff > 60:
            control_motors(-20, 20)  # 우측 회전
        
        time.sleep(0.01)  # 조정 주기
    
    # diff가 적절한 범위에 들어오면 멈춤
    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()
    time.sleep(1)
    
    print(f"ld : {ld}, rd : {rd}")
    
    # 2번과 같은 기존 회피 로직 이어서 수행
    if ld > rd:
        print("좌측 회피")
        control_motors(-50, 50)  # 좌회전
        time.sleep(0.60)  # 회피 시간 설정
        control_motors(40, 40)  # 직진
        time.sleep(1.25)
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()
        time.sleep(0.1)
        control_motors(50, -50)  # 우회전
        time.sleep(0.90)
        control_motors(40, 40)  # 직진
        time.sleep(1.29)
        
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()
        time.sleep(0.1)
        control_motors(-50, 50)  # 좌회전
        time.sleep(0.3)
        
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()
        time.sleep(0.1)        
        
    else:
        print("우측 회피")
        control_motors(50, -50)
        time.sleep(0.60)  # 회피 시간 설정
        control_motors(40, 40)
        time.sleep(1.25)
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()
        time.sleep(0.1)
        control_motors(-50, 50) 
        time.sleep(0.90)
        
        control_motors(40, 40)
        time.sleep(1.29)
        
        motor1.stop()
        motor2.stop()
        motor3.stop()
        motor4.stop()
        time.sleep(0.1)
        
        control_motors(50, -50)  # 좌회전
        time.sleep(0.3)

    return

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
