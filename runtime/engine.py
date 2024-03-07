import serial
import time

# Setup serial connection - replace '/dev/ttyUSB0' with your serial port
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

def control_motors(pwm1_speed, pwm2_speed, turn_direction):
    # Ensure speeds are within a valid range, e.g., 0-100
    pwm1_speed = max(0, min(100, pwm1_speed))
    pwm2_speed = max(0, min(100, pwm2_speed))

    if turn_direction in ['forward', 'left', 'right', 'stop']:
        # Construct the command string including direction and speeds
        command = f"{turn_direction},{pwm1_speed},{pwm2_speed}\n"
        # Send the command over serial
        ser.write(command.encode('utf-8'))
    else:
        print("Invalid direction")