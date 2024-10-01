import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import threading

# PID control coefficients
Kp = 1.000
Ki = 0.000
Kd = 0.000

prev_error = 0
integral = 0
setpoint = 320  # Setpoint for the center of the frame
prev_time = time.time()

# Shared variables between threads
error_lock = threading.Lock()  # Lock for thread-safe access to error
error = 0  # Shared error variable

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
        # Limit the speed between 0 and 100 (PWM duty cycle should not be negative)
        speed = max(min(speed, 100), 0)
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

# PID control function
def pid_control(error, dt):
    global prev_error, integral

    proportional = error
    integral += error * dt
    derivative = (error - prev_error) / dt if dt > 0 else 0  # Prevent division by zero
    prev_error = error

    # Return the PID control result
    return Kp * proportional + Ki * integral + Kd * derivative

# Function to capture and process frames from the camera (Thread 1)
def camera_thread():
    global error

    # Initialize the camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("Error: Cannot open webcam")
        return

    # Set lower resolution for faster processing
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image")
            break

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
            new_error = middle - setpoint  # Calculate error from the center

            # Lock the shared error variable and update it
            with error_lock:
                error = new_error

        else:
            print("Line not detected.")

        time.sleep(0.01)  # Small delay to avoid excessive CPU usage

    cap.release()

# Function to control the motors based on the PID control (Thread 2)
def motor_control_thread():
    global error
    prev_time = time.time()

    while True:
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Lock the shared error variable and get the current error value
        with error_lock:
            current_error = error

        # Compute the PID adjustment
        steering_adjustment = pid_control(current_error, dt)

        # Compute motor speeds
        base_speed = 60
        correction_factor = 1.200
        
        left_motor_speed = base_speed - steering_adjustment
        right_motor_speed = base_speed + steering_adjustment

        # Limit motor speeds to the range [0, 100]
        right_motor_final_speed = max(min(right_motor_speed * correction_factor, 100), 0)
        left_motor_final_speed = max(min(left_motor_speed, 100), 0)

        motor1.forward(right_motor_final_speed)
        motor2.forward(left_motor_final_speed)

        print(f"Left Motor Speed: {left_motor_final_speed}, Right Motor Speed: {right_motor_final_speed}, Error: {current_error}")

        time.sleep(0.01)  # Small delay to avoid excessive CPU usage

# Start the threads
camera_thread = threading.Thread(target=camera_thread)
motor_control_thread = threading.Thread(target=motor_control_thread)

camera_thread.start()
motor_control_thread.start()

try:
    # Wait for both threads to complete
    camera_thread.join()
    motor_control_thread.join()

except KeyboardInterrupt:
    # Clean up on keyboard interrupt
    motor1.cleanup()
    motor2.cleanup()
    GPIO.cleanup()

finally:
    # Clean up resources
    motor1.cleanup()
    motor2.cleanup()
    GPIO.cleanup()
