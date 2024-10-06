import board
import busio
import adafruit_adxl34x
import time

# I2C 초기화
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c, address=0x1D)

# 가속도 데이터 읽기
while True:
    print("X: {0}, Y: {1}, Z: {2}".format(*accelerometer.acceleration))
    time.sleep(1)
