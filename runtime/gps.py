import serial
from ublox_gps import UbloxGps

# Serial port configuration
port = "/dev/ttyGSM1"  # Adjust this according to your GPS module's serial port
baudrate = 115200

# Function to read GPS data using ublox_gps
def read_gps_data_ublox():
    with serial.Serial(port, baudrate=baudrate, timeout=1) as ser:
        gps = UbloxGps(ser)
        try:
            geo = gps.geo_coords()
            latitude = geo.lat
            longitude = geo.lon
            
            veh = gps.veh_attitude()
            roll = veh.roll
            pitch = veh.pitch
            heading = veh.heading
            accRoll = veh.accRoll
            accPitch = veh.accPitch
            accHeading = veh.accHeading
            return latitude, longitude, heading, accHeading
        except Exception as e:
            print(f"An error occurred: {e}")
            return None, None

