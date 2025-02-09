import smbus
import time
import math
import numpy as np
import RPi.GPIO as GPIO

# MPU6050 Registers and Address
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# Servo Pin Definitions
SERVO_PIN_1 = 22
SERVO_PIN_2 = 16

# Initialize I2C
bus = smbus.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # Wake up MPU6050

# Set up GPIO for servo control
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN_1, GPIO.OUT)
GPIO.setup(SERVO_PIN_2, GPIO.OUT)

# Configure PWM for servos
servo1 = GPIO.PWM(SERVO_PIN_1, 50)  # 50Hz PWM
servo2 = GPIO.PWM(SERVO_PIN_2, 50)  # 50Hz PWM
servo1.start(7.5)  # Neutral position (90°)
servo2.start(7.5)

# Kalman filter variables
x_est = 0.0  # Initial estimated angle
P = 1.0  # Initial uncertainty
Q = 0.01  # Process noise
R = 0.1   # Measurement noise

def kalman_filter(measurement):
    """Apply Kalman filter to smooth noisy measurements."""
    global x_est, P, Q, R
    
    # Prediction step (assuming constant motion)
    P = P + Q

    # Update step
    K = P / (P + R)  # Kalman gain
    x_est = x_est + K * (measurement - x_est)
    P = (1 - K) * P

    return x_est

def read_raw_data(addr):
    """Read raw 16-bit data from MPU6050 register."""
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

def get_x_rotation():
    """Calculate rotation around the X-axis."""
    accel_x = read_raw_data(ACCEL_XOUT_H) / 16384.0
    accel_y = read_raw_data(ACCEL_XOUT_H + 2) / 16384.0
    accel_z = read_raw_data(ACCEL_XOUT_H + 4) / 16384.0
    
    x_rotation = math.degrees(math.atan2(accel_y, accel_z))
    return x_rotation

def map_angle(raw_angle):
    """Scale small angle changes into larger servo movements."""
    scaled_angle = 90 + (abs(raw_angle) ** 1.5 / 10) * (1 if raw_angle >= 0 else -1)
    return max(0, min(180, scaled_angle))


def set_servo_angle(servo, angle):
    """Convert angle (0-180) to duty cycle (2.5-12.5) and set PWM signal."""
    duty_cycle = 2.5 + (angle / 180.0) * 10
    servo.ChangeDutyCycle(duty_cycle)

try:
    while True:
        raw_x_rotation = get_x_rotation()
        filtered_x_rotation = kalman_filter(raw_x_rotation)
        servo_angle = map_angle(filtered_x_rotation)

        print(f"Raw X Rotation: {raw_x_rotation:.2f}° | Filtered: {filtered_x_rotation:.2f}° | Servo Angle: {servo_angle:.2f}°")

        set_servo_angle(servo1, servo_angle)
        set_servo_angle(servo2, servo_angle)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping servos and cleaning up GPIO.")
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
