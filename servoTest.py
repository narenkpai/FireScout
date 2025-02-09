import smbus
import time
import math
import numpy as np
import RPi.GPIO as GPIO

# MPU6050 Registers and Address
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# Servo Pin Definition
SERVO_PIN_Y = 12  # Servo for Y-axis

# Initialize I2C
bus = smbus.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # Wake up MPU6050

# Set up GPIO for servo control
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN_Y, GPIO.OUT)

# Configure PWM for servo (50Hz)
servo_y = GPIO.PWM(SERVO_PIN_Y, 50)

# Start servo at neutral position (90°)
servo_y.start(7.5)

# Kalman filter variables
y_est = 0.0  # Initial estimated angle
P_y = 1.0  # Initial uncertainty
Q = 0.005  # Process noise (low for smoother response)
R = 0.2    # Measurement noise (reduces jitter)

# Exponential Moving Average (EMA) smoothing
servo_y_prev = 90  # Start at neutral

def kalman_filter(measurement, est, P):
    """Apply Kalman filter to smooth noisy measurements."""
    global Q, R
    
    P = P + Q  # Prediction step
    K = P / (P + R)  # Kalman gain
    est = est + K * (measurement - est)
    P = (1 - K) * P  # Update step

    return est, P

def read_raw_data(addr):
    """Read raw 16-bit data from MPU6050 register."""
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

def get_y_rotation():
    """Calculate rotation around the Y-axis."""
    accel_x = read_raw_data(ACCEL_XOUT_H) / 16384.0
    accel_z = read_raw_data(ACCEL_XOUT_H + 4) / 16384.0
    
    y_rotation = math.degrees(math.atan2(accel_x, accel_z))
    return y_rotation

def map_angle(raw_angle):
    """Increase small angle sensitivity using nonlinear scaling."""
    if raw_angle < -90:
        raw_angle = -90
    elif raw_angle > 90:
        raw_angle = 90

    # Nonlinear scaling: Exaggerates small changes for better responsiveness
    scaled_angle = 90 + (raw_angle / 90) ** 2 * 45 * (1 if raw_angle >= 0 else -1)

    return max(0, min(180, scaled_angle))

def set_servo_angle(servo, angle):
    """Convert angle (0-180) to duty cycle (2.5-12.5) and set PWM signal."""
    duty_cycle = 2.5 + (angle / 180.0) * 10
    servo.ChangeDutyCycle(duty_cycle*1.1)

try:
    while True:
        raw_y_rotation = get_y_rotation()

        # Apply Kalman filter
        y_est, P_y = kalman_filter(raw_y_rotation, y_est, P_y)

        # Map angles to servo range (no dead zone, higher sensitivity)
        servo_angle_y = map_angle(y_est)

        # Smooth response with Exponential Moving Average (EMA)
        alpha = 0.1  # Lower alpha = more smoothing
        servo_angle_y = alpha * servo_angle_y + (1 - alpha) * servo_y_prev
        servo_y_prev = servo_angle_y  # Update for next loop

        print(f"Y: {y_est:.2f}° | ServoY: {servo_angle_y:.2f}°")

        set_servo_angle(servo_y, servo_angle_y)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping servo and cleaning up GPIO.")
    servo_y.stop()
    GPIO.cleanup()
