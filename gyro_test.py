import smbus
import time

# I2C 버스 설정 (보통 라즈베리파이에서는 버스 1)
bus = smbus.SMBus(1)

# I2C 장치 주소 (i2cdetect 명령어에서 확인된 주소)
DEVICE_ADDRESS = 0x1d  # 예: 0x1d

# 장치 레지스터 주소 (예시: 가속도계의 데이터 레지스터)
DEVICE_REG = 0x32  # 읽고 싶은 레지스터 주소

def read_data():
    try:
        # 장치로부터 6바이트의 데이터 읽기 (예: 가속도계 X, Y, Z 데이터)
        data = bus.read_i2c_block_data(DEVICE_ADDRESS, DEVICE_REG, 6)
        print(f"Data: {data}")
        return data
    except IOError:
        print("I2C 통신에 실패했습니다.")
        return None

# 데이터 읽기 반복
while True:
    read_data()
    time.sleep(1)
