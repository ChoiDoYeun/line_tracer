import cv2
import os
import pygame
from datetime import datetime

# pygame 초기화
pygame.init()
pygame.display.set_mode((100, 100))

# 웹캠 초기화
cap = cv2.VideoCapture(0)

# 해상도 설정 (640x240)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# 캡처된 이미지를 저장할 폴더 설정
save_dir = 'data'
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

print("스페이스바를 눌러 캡처하세요. 'q'를 눌러 종료하세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 읽을 수 없습니다.")
        break

    cv2.imshow('Webcam', frame)

    # pygame 이벤트 처리
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:  # 스페이스바 입력
                # 파일명은 현재 시간을 사용하여 고유하게 만듦
                filename = os.path.join(save_dir, datetime.now().strftime("%Y%m%d_%H%M%S") + '.jpg')
                cv2.imwrite(filename, frame)
                print(f"{filename} 저장됨")
            elif event.key == pygame.K_q:  # 'q'를 누르면 종료
                cap.release()
                cv2.destroyAllWindows()
                pygame.quit()
                exit()

    # 'q'를 누르면 종료 (OpenCV에서 직접 처리)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()
pygame.quit()
