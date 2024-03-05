import Jetson.GPIO as GPIO
import time

# Pin Definitions - Update these constants based on your connections
PWM1_PIN = 12  # GPIO07, for Motor 1 PWM (pwm1)
DIR1_PIN = 17 # GPIO01, for Motor 1 Direction (pwm5)
PWM2_PIN = 13  # GPIO13, for Motor 2 PWM (pwm7)
DIR2_PIN = 11  # GPIO11, for Motor 2 Direction (pwm)

# PWM Frequency
PWM_FREQ = 100  # 100 Hz

def setup():
    # Pin Setup
    GPIO.setmode(GPIO.BCM)  # Use BOARD pin numbering
    GPIO.setup(DIR1_PIN, GPIO.OUT)
    GPIO.setup(DIR2_PIN, GPIO.OUT)

    # PWM Setup
    GPIO.setup(PWM1_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PWM2_PIN, GPIO.OUT, initial=GPIO.LOW)
    pwm1 = GPIO.PWM(PWM1_PIN, PWM_FREQ)  # Initialize PWM on PWM1_PIN 100Hz frequency
    pwm2 = GPIO.PWM(PWM2_PIN, PWM_FREQ)  # Initialize PWM on PWM2_PIN 100Hz frequency
    
    return pwm1, pwm2

def control_motors(pwm1, pwm2, turn_direction):
    if turn_direction == 'forward':
        GPIO.output(DIR1_PIN, GPIO.HIGH)
        GPIO.output(DIR2_PIN, GPIO.HIGH)
        pwm1.start(50)
        pwm2.start(50)
    elif turn_direction == 'left':
        pwm1.stop()
        GPIO.output(DIR2_PIN, GPIO.HIGH)
        pwm2.start(50)  # Adjust speed for smoother turning if needed
    elif turn_direction == 'right':
        GPIO.output(DIR1_PIN, GPIO.HIGH)
        pwm1.start(50)  # Adjust speed for smoother turning if needed
        pwm2.stop()
    elif turn_direction == 'stop':
        pwm1.stop()
        pwm2.stop()
    else:
        print("Invalid direction")
        pwm1.stop()
        pwm2.stop()

def cleanup(pwm1, pwm2):
    pwm1.stop()  # Stop PWM
    pwm2.stop()
    GPIO.cleanup()  # cleanup all GPIO