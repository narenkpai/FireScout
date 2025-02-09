import smbus
import time

# MPU6050 I2C address
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
TEMP_OUT_H = 0x41  # Register for temperature data

# Initialize I2C bus
bus = smbus.SMBus(1)

def mpu6050_init():
    """Initialize MPU6050 by waking it up from sleep mode."""
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_raw_data(addr):
    """Read two bytes of data from given register address."""
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def read_temperature():
    """Read temperature from MPU6050 and convert it to Celsius."""
    temp_raw = read_raw_data(TEMP_OUT_H)
    temp_c = (temp_raw / 340.0) + 36.53  # Convert to Celsius
    return temp_c

# Initialize MPU6050
mpu6050_init()

# Loop to read and print temperature every second
while True:
    temp = read_temperature()
    print(f"MPU6050 Temperature: {temp:.2f}C")
    time.sleep(1)  # Wait 1 second before reading again
