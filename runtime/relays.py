import smbus
import time

# Create an instance of the smbus library
bus = smbus.SMBus(7)  # 1 indicates /dev/i2c-1

# I2C address of the device
DEVICE_ADDRESS = 0x27

def set_relay_state(relay_number, state):
    # Calculate the command byte based on the relay number
    command_byte = 0xFF ^ (1 << (relay_number - 1))

    try:
        if state:
            # Turn on the specified relay
            bus.write_byte(DEVICE_ADDRESS, command_byte)
            print(f"Relay {relay_number} activated")
        else:
            # Turn off all relays
            bus.write_byte(DEVICE_ADDRESS, 0xFF)
            print(f"Relay {relay_number} deactivated")
    except Exception as e:
        print(f"Error: {e}")