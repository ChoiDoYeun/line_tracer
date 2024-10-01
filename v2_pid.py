import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# PID control coefficients
Kp = 0.200
Ki = 0.000
Kd = 0.000

prev_error = 0
integral = 0
setpoint = 160  # Adjusted for smaller ROI
prev_time = time.time()

GPIO.setmode(GPIO.BCM)

class MotorController:
    def __init__(self, en, in1, in2):
        self.en = en
        self.in1 = in1
        self.in2 = in2
        GPIO.setup([self.en, self.in1, self.in2], GPIO.OUT)
        self.pwm = GPIO.PWM(self.en, 100)
        self.pwm.start(0)

    def set_speed(self, speed):
        # Limit the speed between -10 and 100
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
        # Stop the motor
        self.set_speed(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def cleanup(self):
        # Cleanup GPIO and PWM
        self.pwm.stop()
        GPIO.cleanup([self.en, self.in1, self.in2])

# Initialize motors
motor1 = MotorController(18, 17, 27)  # Right motor
motor2 = MotorController(16, 13, 26)  # Left motor

# Initialize the camera
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("Error: Cannot open webcam")
    exit()

# Set lower resolution for faster processing
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

def pid_control(error, dt):
    global prev_error, integral

    proportional = error
    integral += error * dt
    derivative = (error - prev_error) / dt
    prev_error = error

    # Return the PID control result
    return Kp * proportional + Ki * integral + Kd * derivative

try:
    while True:
        start_time = time.time()  # Measure frame processing start time
        
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image")
            break

        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time 

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply adaptive thresholding to create a binary image
        thresh = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
        )

        # Detect the line on the bottom row of the image
        start_height = thresh.shape[0] - 5
        signed_thresh = thresh[start_height].astype(np.int16)
        diff = np.diff(signed_thresh)
        points = np.where(np.logical_or(diff > 200, diff < -200))

        if len(points[0]) > 1:
            # Calculate the center of the detected line
            middle = (points[0][0] + points[0][1]) // 2
            error = middle - setpoint  # Calculate error from the center

            # Compute the PID adjustment
            current_time = time.time()
            dt = current_time - prev_time if 'prev_time' in locals() else 0.1
            prev_time = current_time

            steering_adjustment = pid_control(error, dt)

            # Compute motor speeds
            base_speed = 60
            left_motor_speed = base_speed - steering_adjustment
            right_motor_speed = base_speed + steering_adjustment

            # Limit motor speeds to the range [-100, 100]
            right_motor_final_speed = max(min(right_motor_speed, 100), -100)
            left_motor_final_speed = max(min(left_motor_speed - 15, 100), -100)

            # Control the right motor
            if right_motor_final_speed >= 0:
                motor1.forward(right_motor_final_speed)
            else:
                motor1.backward(abs(right_motor_final_speed))

            # Control the left motor
            if left_motor_final_speed >= 0:
                motor2.forward(left_motor_final_speed)
            else:
                motor2.backward(abs(left_motor_final_speed))

            print(f"Left Motor Speed: {left_motor_final_speed}, Right Motor Speed: {right_motor_final_speed}, Error: {error}")
        else:
            # Stop the motors if no line is detected
            motor1.stop()
            motor2.stop()
            print("Line not detected. Motors stopped.")

        end_time = time.time()
        print(f"Frame processing time: {end_time - start_time:.4f} seconds")  # Print frame processing time

except KeyboardInterrupt:
    # Clean up on keyboard interrupt
    motor1.cleanup()
    motor2.cleanup()
    GPIO.cleanup()
    cap.release()

finally:
    # Clean up resources
    motor1.cleanup()
    motor2.cleanup()
    GPIO.cleanup()
    cap.release()
