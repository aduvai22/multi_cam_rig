#!/usr/bin/env python3
import time
from smbus2 import SMBus

BUS_NUM = 1
ISM_ADDR = 0x6B  # common; if i2cdetect shows 0x6A, change this to 0x6A

# ISM330DHCX registers (compatible with LSM6DSO family map)
WHO_AM_I = 0x0F

CTRL1_XL = 0x10
CTRL2_G  = 0x11
CTRL3_C  = 0x12

OUT_TEMP_L = 0x20  # temp LSB
OUTX_L_G   = 0x22  # gyro X LSB
OUTX_L_A   = 0x28  # accel X LSB

# Scales based on configuration below:
# Accel: ±2g => 16384 LSB/g
ACCEL_SCALE = 9.80665 / 16384.0  # m/s^2 per count

# Gyro: ±250 dps => 131.072 LSB/(dps)
# Convert dps -> rad/s: (pi/180)
GYRO_SCALE = (1.0 / 131.072) * (3.141592653589793 / 180.0)  # rad/s per count

def to_int16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v

def write_reg(bus, reg, val):
    bus.write_byte_data(ISM_ADDR, reg, val)

def read_reg(bus, reg):
    return bus.read_byte_data(ISM_ADDR, reg)

def read_block(bus, start_reg, length):
    return bus.read_i2c_block_data(ISM_ADDR, start_reg, length)

def init_ism330(bus):
    # Check WHO_AM_I
    who = read_reg(bus, WHO_AM_I)
    print(f"WHO_AM_I = 0x{who:02X} (common expected is 0x6B for ISM330DHCX)")
    if who not in (0x6B, 0x6A):
        print("WARNING: WHO_AM_I not matching typical value; wiring/address may be wrong.")

    # Soft reset
    write_reg(bus, CTRL3_C, 0x01)
    time.sleep(0.05)

    # CTRL3_C: BDU=1 (block data update), IF_INC=1 (auto-increment)
    # BDU bit is 6, IF_INC bit is 2 => 0x40 + 0x04 = 0x44
    write_reg(bus, CTRL3_C, 0x44)
    time.sleep(0.01)

    # CTRL1_XL: ODR_XL=104 Hz, FS_XL=±2g
    # For this family, 0x40 typically sets 104Hz, and 0x00 FS=2g; 0x48 is a common working combo.
    write_reg(bus, CTRL1_XL, 0x40)

    # CTRL2_G: ODR_G=104 Hz, FS_G=±250 dps
    # 0x40 often = 104Hz, 0x00 FS=250dps; 0x44 is a common working combo.
    write_reg(bus, CTRL2_G, 0x40)

def read_acc_gyro_temp(bus):
    # Read temp (2 bytes) + gyro (6 bytes) + accel (6 bytes) in one shot:
    # start at OUT_TEMP_L (0x20), total 14 bytes through accel Z high
    b = read_block(bus, OUT_TEMP_L, 14)

    temp_raw = to_int16(b[0], b[1])

    gx_raw = to_int16(b[2], b[3])
    gy_raw = to_int16(b[4], b[5])
    gz_raw = to_int16(b[6], b[7])

    ax_raw = to_int16(b[8],  b[9])
    ay_raw = to_int16(b[10], b[11])
    az_raw = to_int16(b[12], b[13])

    # Convert
    ax = ax_raw * ACCEL_SCALE
    ay = ay_raw * ACCEL_SCALE
    az = az_raw * ACCEL_SCALE

    gx = gx_raw * GYRO_SCALE
    gy = gy_raw * GYRO_SCALE
    gz = gz_raw * GYRO_SCALE

    # Temperature conversion (common for this family):
    # Temp in °C = 25 + temp_raw/256
    temp_c = 25.0 + (temp_raw / 256.0)

    return ax, ay, az, gx, gy, gz, temp_c

def main():
    print("Opening I2C bus", BUS_NUM)
    with SMBus(BUS_NUM) as bus:
        init_ism330(bus)
        print("\nPrinting acc (m/s^2), gyro (rad/s), temp (C). Ctrl+C to stop.\n")

        # Print at ~10 Hz for easy viewing
        period = 0.1
        next_t = time.monotonic()

        while True:
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(0.01, next_t - now))
                continue
            next_t += period

            try:
                ax, ay, az, gx, gy, gz, temp_c = read_acc_gyro_temp(bus)
                print(
                    f"ACC  [{ax:8.3f} {ay:8.3f} {az:8.3f}] m/s^2  | "
                    f"GYRO [{gx:8.4f} {gy:8.4f} {gz:8.4f}] rad/s  | "
                    f"T={temp_c:6.2f} C"
                )
            except Exception as e:
                print("Read error:", e)
                time.sleep(0.2)

if __name__ == "__main__":
    main()
