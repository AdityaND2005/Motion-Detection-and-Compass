#Libraries
from time import sleep_ms, ticks_ms, ticks_diff
from machine import Pin, I2C, TouchPad
from micropython import const
from ssd1306 import SSD1306_I2C
from mpu9250 import MPU9250
from math import sqrt, atan2, pi, copysign, sin, cos

#Constants
SCREEN_WIDTH = 128
SCREEN_HEIGHT = 64
TOUCH_THRESHOLD = 300
TOUCH_PIN = Pin(14)
SDA = Pin(1)
SCL = Pin(2)
DISPLAY_MODE_ACCEL = 0
DISPLAY_MODE_GYRO = 1
DISPLAY_MODE_MAG = 2
display_mode = DISPLAY_MODE_ACCEL

#Touch Control
last_touch_time = 0
debounce_time = 300 #ms

#Bias offset
pitch_bias = 0.0
roll_bias = 0.0

#For low pass filtering
filtered_x_value = 0.0
filtered_y_value = 0.0

#I2C Bus
i2c = I2C(1, scl=SCL, sda=SDA)

#Touch
touch = TouchPad(TOUCH_PIN)

#Initializing SSD1306 and MPU9250
oled = SSD1306_I2C(SCREEN_WIDTH, SCREEN_HEIGHT, i2c)
m = MPU9250(i2c)
print("I2C devices found:", [hex(addr) for addr in i2c.scan()])

#Accelerometer
def detect_movement():
    x = m.acceleration[0] 
    y = m.acceleration[1]
    z = m.acceleration[2] 
    # Detect linear movement
    if abs(x) > 1.0:
        return "Left" if x < 0 else "Right"
    elif abs(y) > 1.0:
        return "Backward" if y < 0 else "Forward"
    elif abs(z - 9.8) > 1.0:  # gravity (~9.8 m/s²)
        return "Down" if z < 9.8 else "Up" 
    return "Still"

#Gyroscope
def detect_orientation():
    x = m.gyro[0]
    y = m.gyro[1]
    z = m.gyro[2]
    # Detect rotation
    pitch_rate = "Pitch " + ("Up" if x > 0.5 else "Down" if x < -0.5 else "Stable")
    roll_rate = "Roll " + ("Right" if y > 0.5 else "Left" if y < -0.5 else "Stable")
    yaw_rate = "Yaw " + ("Right" if z > 0.5 else "Left" if z < -0.5 else "Stable")    
    return pitch_rate, roll_rate, yaw_rate

#Magnetometer
def low_pass_filter(raw_value:float, remembered_value):
    alpha = 0.8
    filtered = 0
    filtered = (alpha * remembered_value) + (1.0 - alpha) * raw_value
    return filtered

def degrees_to_heading(degrees):
    heading = ""
    if (degrees > 337) or (degrees >= 0 and degrees <= 22):
            heading = 'N'
    if degrees >22 and degrees <= 67:
        heading = "NE"
    if degrees >67 and degrees <= 112:
        heading = "E"
    if degrees >112 and degrees <= 157:
        heading = "SE"
    if degrees > 157 and degrees <= 202:
        heading = "S"
    if degrees > 202 and degrees <= 247:
        heading = "SW"
    if degrees > 247 and degrees <= 292:
        heading = "W"
    if degrees > 292 and degrees <= 337:
        heading = "NW"
    return heading

def get_reading()->float:
    global filtered_y_value, filtered_x_value
    x = m.acceleration[0] 
    y = m.acceleration[1]
    z = m.acceleration[2] 

    roll_rad = atan2(-x, sqrt((z*z)+(y*y)))
    pitch_rad = atan2(z, copysign(y,y)*sqrt((0.01*x*x)+(y*y)))

    # Pitch and Roll in Degrees
    pitch = pitch_rad*180/pi
    roll = roll_rad*180/pi

    mag_x, mag_y, magz = m.magnetic

    filtered_x_value = low_pass_filter(mag_x, filtered_x_value)
    filtered_y_value = low_pass_filter(mag_y, filtered_y_value)

    az =  90 - atan2(filtered_y_value, filtered_x_value) * 180 / pi

    if az < 0:
        az += 360
        
    pitch -= pitch_bias
    roll -= roll_bias

    heading = degrees_to_heading(az)
    return pitch, roll, az, heading

#Calibration
def calibrate():
    oled.fill(0)
    oled.text("Calibrating...", 0, 0)
    oled.text("Keep still", 0, 10)
    oled.show()
    m.mpu6500.calibrate()
    oled.text("Gyro done", 0, 20)
    oled.show()
    sleep_ms(500)
    oled.text("Rotate device", 0, 30)
    oled.show()
    m.ak8963.calibrate(count=100)
    global pitch_bias, roll_bias
    pitch_bias, roll_bias, _, _ = get_reading()
    oled.text("Calibration", 0, 40)
    oled.text("complete!", 0, 50)
    oled.show()
    sleep_ms(1000)
    
#MAIN
while True:
    touch_value = touch.read()
    current_time = ticks_ms()
    if touch_value < TOUCH_THRESHOLD and ticks_diff(current_time, last_touch_time) > debounce_time:
        display_mode = (display_mode + 1) % 3
        last_touch_time = current_time
        oled.fill(0)
        mode_name = ["Accelerometer", "Gyroscope", "Compass"][display_mode]
        oled.text(f"Mode: {mode_name}", 0, 30)
        oled.show()
        time.sleep(1)
        
    oled.fill(0)
    
    if display_mode == DISPLAY_MODE_ACCEL:
        movement = detect_movement()
        oled.text("Accelerometer:", 0, 0)
        oled.text(f"Move: {movement}", 0, 15)
        
    elif display_moder == DISPLAY_MODE_GYRO:
        p_r, r_r, y_r = detect_orientation()
        oled.text("Gyroscope:", 0, 0)
        oled.text(pitch_rate, 0, 15)
        oled.text(roll_rate, 0, 30)
        oled.text(yaw_rate, 0, 45)
        
    else:
        _,_, az, heading = get_reading()
        oled.text("Compass:", 0, 0)
        oled.text(f"Heading: {heading}", 0, 10)
        oled.text(f"Degrees: {round(az,1)}", 0, 20)
        
        # Compass indicator
        center_x, center_y = 65, 46
        radius = 15
        angle_rad = (az * pi / 180)
        end_x = int(center_x + radius * cos(angle_rad))
        end_y = int(center_y - radius * sin(angle_rad))
            
        # Draw circle
        for angle in range(0, 360, 30):
            a = angle * pi / 180
            x = int(center_x + radius * cos(a))
            y = int(center_y - radius * sin(a))
            oled.pixel(x, y, 1)
            
        # Draw pointer
        oled.line(center_x, center_y, end_x, end_y, 1)
        
    oled.show()
    sleep_ms(100)