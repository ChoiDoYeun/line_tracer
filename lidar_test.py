import smbus
import time

# I2C 버스 번호 및 센서 주소 설정
bus = smbus.SMBus(21)  # /dev/i2c-20 버스 사용
address = 0x10  # 인식된 TF Luna 센서 주소

def read_distance():
    try:
        # 거리 데이터를 읽음 (센서에 따라 레지스터 주소가 다를 수 있음)
        data = bus.read_i2c_block_data(address, 0x00, 2)
        distance = data[0] + (data[1] << 8)
        return distance
    except OSError as e:
        print(f"Error reading from sensor: {e}")
        return None

while True:
    distance = read_distance()
    if distance:
        print(f"Distance: {distance} cm")
    time.sleep(0.5)
