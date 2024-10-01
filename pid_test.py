import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

Kp = 0.200
Ki = 0.000
Kd = 0.001

prev_error = 0
integral = 0
setpoint = 320
prev_time = time.time()

GPIO.setmode(GPIO.BCM)

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
        speed = max(min(speed, 100), -10)
        self.pwm.ChangeDutyCycle(speed)

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

motor1 = MotorController(18, 17, 27)
motor2 = MotorController(16, 13, 26)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("Error: Cannot open webcam")
    exit()

def pid_control(error, dt):
    global prev_error, integral

    proportional = error
    integral += error * dt
    derivative = (error - prev_error) / dt
    prev_error = error

    return Kp * proportional + Ki * integral + Kd * derivative

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image")
            break

        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time 

        height, width = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_black = np.array([0, 0, 0])       # H: 0, S: 0, V: 0
        upper_black = np.array([200, 255, 100])

        # Create a mask to detect black regions
        mask = cv2.inRange(hsv, lower_black, upper_black)
        
        roi = mask[height//2:height, :]

        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M['m00'] > 0:
                middle = int(M['m10'] / M['m00'])
                error = middle - setpoint

                base_speed = 60
                left_motor_speed = base_speed - pid_control(error, dt)
                right_motor_speed = base_speed + pid_control(error, dt)
                
                right_motor_final_speed = max(min(right_motor_speed + 15, 100), -100)
                left_motor_final_speed = max(min(left_motor_speed, 100), -100)

                if right_motor_final_speed >= 0:
                    motor1.forward(right_motor_final_speed)
                else:
                    motor1.backward(abs(right_motor_final_speed)) 

                if left_motor_final_speed >= 0:
                    motor2.forward(left_motor_final_speed)
                else:
                    motor2.backward(abs(left_motor_final_speed))
                
                print(f"Left Motor Speed: {left_motor_final_speed}, Right Motor Speed: {right_motor_final_speed}, Error: {error}")
            else:
                motor1.stop()
                motor2.stop()
                print("Line not detected. Motors stopped.")
 

except KeyboardInterrupt:
    motor1.cleanup()
    motor2.cleanup()
    GPIO.cleanup()
    cap.release()
finally:
    motor1.cleanup()
    motor2.cleanup()
    GPIO.cleanup()
    cap.release()
