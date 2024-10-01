import cv2
import numpy as np
import os
from glob import glob
import time
import multiprocessing as mp

# 이미지 경로 설정
image_dir = "/home/dodo/line_tracer/data/"

def process_image(image_path):
    # 이미지 읽기
    sample_img = cv2.imread(image_path)

    # 이미지를 HLS로 변환
    hls = cv2.cvtColor(sample_img, cv2.COLOR_BGR2HLS)

    # S 채널 추출
    s_channel = hls[:, :, 2]

    # Gaussian 블러 적용
    blurred = cv2.GaussianBlur(s_channel, (5, 5), 0)

    # Canny 에지 감지 적용
    canny_edges = cv2.Canny(blurred, 50, 150)

    # Hough Line Transform 적용
    lines = cv2.HoughLinesP(canny_edges, 1, np.pi / 180, threshold=50, minLineLength=10, maxLineGap=10)

    # 선을 그릴 이미지 복사본 생성
    line_img = sample_img.copy()

    # y = 240에서부터 5 단위로 줄여가며 선을 검출
    found = False
    for y in range(240, 119, -5):  # y=240부터 y=120까지 5 단위로 올라감
        x_positions = []

        if lines is not None:
            # 각 라인에서 해당 y 좌표에 대한 x 값을 찾음
            for line in lines:
                x1, y1, x2, y2 = line[0]

                if y1 <= y <= y2 or y2 <= y <= y1:  # y 좌표가 라인의 범위 안에 있는지 확인
                    # 라인의 방정식을 사용하여 주어진 y에서 x 좌표 계산
                    x = int(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
                    x_positions.append(x)

            # 두 선이 감지되었다면, 두 선 사이의 중앙값을 계산
            if len(x_positions) == 2:
                left_x, right_x = sorted(x_positions)
                line_center_x = (left_x + right_x) // 2
                diff = line_center_x - 211

                print(f"y = {y}에서 두 선 사이의 중앙값: {line_center_x}")
                print(f"중앙값과 211의 차이: {diff}")

                found = True

        # 찾았다면 더 이상 위로 올라가지 않음
        if found:
            break

    return line_center_x, diff

# 모든 jpg 파일 경로 가져오기
image_paths = glob(os.path.join(image_dir, "*.jpg"))

### 단일 프로세싱 시간 측정 ###
start_time_single = time.perf_counter()

for image_path in image_paths:
    process_image(image_path)

end_time_single = time.perf_counter()

# 단일 프로세싱 평균 FPS 계산
total_time_single = end_time_single - start_time_single
fps_single = len(image_paths) / total_time_single if total_time_single > 0 else 0
print(f"Single Processing FPS: {fps_single:.6f}")

### 멀티프로세싱 시간 측정 ###
start_time_multi = time.perf_counter()

# 멀티프로세싱을 이용한 이미지 처리
pool = mp.Pool(mp.cpu_count())
processed_images = pool.map(process_image, image_paths)

end_time_multi = time.perf_counter()

# 멀티프로세싱 평균 FPS 계산
total_time_multi = end_time_multi - start_time_multi
fps_multi = len(image_paths) / total_time_multi if total_time_multi > 0 else 0
print(f"Multi Processing FPS: {fps_multi:.6f}")
