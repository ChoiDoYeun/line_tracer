from mpu6050 import mpu6050
import time

# 센서 초기화
sensor = mpu6050(0x1d)

while True:
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    
    print(f"Accelerometer: {accel_data}")
    print(f"Gyroscope: {gyro_data}")
    
    time.sleep(1)
