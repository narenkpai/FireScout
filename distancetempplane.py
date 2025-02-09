import pygame
import math
import smbus
import time
import RPi.GPIO as GPIO  # Library for ultrasonic sensor

# Initialize Pygame
pygame.init()

# Screen settings
WIDTH, HEIGHT = 600, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Rotating Rectangle Controlled by MPU6050 with Sensor Display")

# Font for displaying text
font = pygame.font.Font(None, 36)

# Plane shape (rectangle)
plane_vertices = [
    (-50, -25), (50, -25), (50, 25), (-50, 25)  # Rectangle shape
]

# MPU6050 Setup
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
TEMP_OUT_H = 0x41  # Temperature register

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
    temp_c = (temp_raw / 340.0) + 36.53  # Convert raw data to Celsius
    return temp_c

# Kalman Filter Class
class KalmanFilter:
    def __init__(self, process_variance=0.01, measurement_variance=0.1):
        self.x = 0  # Estimated angle
        self.p = 1  # Estimated error covariance
        self.q = process_variance  # Process variance (small changes)
        self.r = measurement_variance  # Measurement variance (sensor noise)
        self.k = 0  # Kalman gain

    def update(self, measurement):
        """Apply Kalman filter to smooth the motion data."""
        # Prediction
        self.p += self.q
        
        # Update
        self.k = self.p / (self.p + self.r)
        self.x += self.k * (measurement - self.x)
        self.p *= (1 - self.k)
        
        return self.x  # Return the filtered angle

# Initialize MPU6050
mpu6050_init()

# Initialize Kalman filter for rotation
kalman_x = KalmanFilter(process_variance=0.01, measurement_variance=0.1)

# Rotation angle
angle_x = 0
dt = 0.1  # Time step for integration

# Ultrasonic Sensor Setup
TRIG_PIN = 4
ECHO_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def read_distance():
    """Read distance from HC-SR04 ultrasonic sensor with timeout to prevent freezing."""
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # Send a short 10us pulse to trigger
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    timeout = start_time + 0.02  # 20ms timeout

    # Wait for echo start with timeout
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()
        if time.time() > timeout:
            return None  # Timeout, return None to indicate failure

    stop_time = time.time()
    timeout = stop_time + 0.02  # 20ms timeout for echo response

    # Wait for echo return with timeout
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()
        if time.time() > timeout:
            return None  # Timeout, return None to indicate failure

    # Calculate distance in cm
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Speed of sound in air is 343 m/s

    return round(distance, 2)  # Round to two decimal places

def rotate_point(point, angle):
    """Rotate a point around the center by a given angle (in degrees)."""
    rad = math.radians(angle)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    x_new = point[0] * cos_a - point[1] * sin_a
    y_new = point[0] * sin_a + point[1] * cos_a
    return x_new, y_new

def draw_plane(angle_x):
    """Draw a rotating 2D rectangle."""
    rotated_points = [rotate_point(p, angle_x) for p in plane_vertices]

    # Convert to screen coordinates
    screen_points = [(WIDTH // 2 + int(p[0]), HEIGHT // 2 + int(p[1])) for p in rotated_points]

    pygame.draw.polygon(screen, (255, 255, 255), screen_points, 2)  # Draw the rectangle

running = True
clock = pygame.time.Clock()

while running:
    screen.fill((0, 0, 0))  # Clear screen

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    try:
        # Read gyroscope data from MPU6050
        gyro_x = read_raw_data(GYRO_XOUT_H) / 131.0  # Convert raw to deg/s

        # Apply Kalman filter
        filtered_x = kalman_x.update(gyro_x)

        # Integrate to get rotation angle
        angle_x += filtered_x * dt

        # Read temperature from MPU6050
        temperature = read_temperature()
        temp_text = f"MPU6050 Temp: {temperature:.1f}°C"

        # Read distance from Ultrasonic Sensor
        distance = read_distance()
        distance_text = "Distance: No Signal" if distance is None else f"Distance: {distance:.2f} cm"

        # Draw rotating rectangle
        draw_plane(angle_x)

        # Render text
        temp_surface = font.render(temp_text, True, (255, 255, 255))
        distance_surface = font.render(distance_text, True, (255, 255, 255))

        # Display text
        screen.blit(temp_surface, (10, 10))
        screen.blit(distance_surface, (10, 50))  # Below temperature text

        pygame.display.flip()
        clock.tick(30)  # Limit FPS

    except Exception as e:
        print(f"Error: {e}")
        running = False

# Cleanup GPIO
GPIO.cleanup()
pygame.quit()
