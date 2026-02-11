import smbus
import time

# ICM20948 Constants
I2C_ADDR = 0x68
REG_WHO_AM_I = 0x00
CHIP_ID = 0xEA  # The expected ID for ICM20948

bus = smbus.SMBus(1)

try:
    who_am_i = bus.read_byte_data(I2C_ADDR, REG_WHO_AM_I)
    if who_am_i == CHIP_ID:
        print(f"Success! Found ICM20948. ID: {hex(who_am_i)}")
    else:
        print(f"Communication okay, but ID {hex(who_am_i)} doesn't match {hex(CHIP_ID)}.")
except Exception as e:
    print(f"Error: {e}")