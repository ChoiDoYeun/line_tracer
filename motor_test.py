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
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in1, GPIO.HIGH)
        
    def backward(self, speed=40):
        self.set_speed(speed)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in1, GPIO.LOW)   

    def stop(self):
        self.set_speed(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup([self.en, self.in1, self.in2])


motor1 = MotorController(18, 17, 27)
motor2 = MotorController(22, 23, 24)
motor3 = MotorController(9, 10, 11)
motor4 = MotorController(25, 8, 7)


try:
    motor1.forward(80)
    motor2.backward(80)
    motor3.forward(80)
    motor4.backward(80)
    time.sleep(2)
    
    # 모든 모터 정지
    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()
    

except KeyboardInterrupt:
    motor1.cleanup()
    motor2.cleanup()
    motor3.cleanup()
    motor4.cleanup()
    GPIO.cleanup()
    
finally:
    motor1.cleanup()
    motor2.cleanup()
    motor3.cleanup()
    motor4.cleanup()
    GPIO.cleanup()
