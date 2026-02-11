#!/usr/bin/env python3

import time
import board
import busio
from PIL import Image, ImageDraw
import adafruit_ssd1306

# -----------------------------
# Initialize I2C + display
# -----------------------------
i2c = busio.I2C(board.SCL, board.SDA)

# Most 0.91" OLEDs are 128x32
WIDTH = 128
HEIGHT = 32

disp = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c)
disp.fill(0)
disp.show()

# Image buffer
image = Image.new("1", (WIDTH, HEIGHT))
draw = ImageDraw.Draw(image)

print("OLED initialized. Starting bar test...")

# -----------------------------
# Simple moving bar animation
# -----------------------------
bar_width = 20
x = 0
direction = 1

while True:
    draw.rectangle((0, 0, WIDTH, HEIGHT), fill=0)

    # Draw bar
    draw.rectangle((x, 8, x + bar_width, 24), fill=255)

    disp.image(image)
    disp.show()

    x += direction * 4
    if x <= 0 or x + bar_width >= WIDTH:
        direction *= -1

    time.sleep(0.05)
