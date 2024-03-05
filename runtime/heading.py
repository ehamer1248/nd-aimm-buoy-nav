import smbus2
import math

# I2C configuration
bus_number = 1  # I2C bus number the sensor is connected to (usually 1 on newer boards)
device_address = 0x1E  # HMC5883L device address

# HMC5883L register addresses
config_reg_a = 0x00
config_reg_b = 0x01
mode_reg = 0x02
data_reg_begin = 0x03

# Initialize I2C (SMBus)
bus = smbus2.SMBus(bus_number)

# Function to initialize HMC5883L sensor
def init_hmc5883l():
    # Write to Configuration Register A: 0x70 (8-average, 15 Hz default, normal measurement)
    bus.write_byte_data(device_address, config_reg_a, 0x70)

    # Write to Configuration Register B: 0xA0 (Gain = 5)
    bus.write_byte_data(device_address, config_reg_b, 0xA0)

    # Write to mode register: 0x00 (Continuous-Measurement Mode)
    bus.write_byte_data(device_address, mode_reg, 0x00)

# Function to read data from HMC5883L sensor and calculate heading
def read_heading():
    # Read data from data register
    data = bus.read_i2c_block_data(device_address, data_reg_begin, 6)

    # Convert data to 16-bit values
    x = data[0] << 8 | data[1]
    z = data[2] << 8 | data[3]
    y = data[4] << 8 | data[5]

    # Handle negative values
    if x > 32767:
        x -= 65536
    if y > 32767:
        y -= 65536
    if z > 32767:
        z -= 65536

    # Calculate heading
    heading = math.atan2(y, x)
    
    # Correct for when signs are reversed
    if heading < 0:
        heading += 2 * math.pi

    # Convert to degrees
    heading_degrees = math.degrees(heading)
    return heading_degrees

# Initialize sensor
init_hmc5883l()

# Read current heading
current_heading = read_heading()
