import RPi.GPIO as GPIO
import time

# GPIO 핀 설정
servo_pin = 4  # 서보모터를 연결할 GPIO 핀 번호

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM 주파수 설정 (서보모터의 주파수는 보통 50Hz)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    # 각도를 서보모터의 듀티사이클로 변환 (0도에서 180도 사이)
    duty = 2 + (angle / 18)
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        # 0도, 90도, 180도로 이동
        set_angle(0)
        print("0도")
        time.sleep(2)
        set_angle(90)
        print("90도")
        time.sleep(2)
        set_angle(180)
        print("180도")
        time.sleep(2)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
