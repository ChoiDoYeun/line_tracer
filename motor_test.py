import RPi.GPIO as GPIO
import time

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


motor1 = MotorController(18, 17, 27) #right motor
motor2 = MotorController(16, 13, 26) # left motor


try:
    motor1.forward(50)
    motor2.stop()
    time.sleep(1)
    
    motor2.forward(50)
    motor1.stop()
    time.sleep(1)

    motor1.backward(50)
    motor2.backward(50)
    time.sleep(5)
    
    print("motor stop")
    motor1.stop()
    motor2.stop()
except KeyboardInterrupt:
    motor1.cleanup()
    motor2.cleanup()    
    GPIO.cleanup()
    
finally:

    motor1. cleanup()
    motor2.cleanup()
    GPIO.cleanup()
