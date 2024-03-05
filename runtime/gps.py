import serial
import pynmea2

# Serial port configuration
port = "/dev/ttyGSM1"  # Adjust this according to your GPS module's serial port
baudrate = 115200

# Function to read GPS data
def read_gps_data():
    with serial.Serial(port, baudrate=baudrate, timeout=1) as ser:
        # Read data from GPS
        data = ser.readline()

        # Attempt to parse NMEA sentence
        try:
            msg = pynmea2.parse(data.decode('ascii', errors='replace'))
            if isinstance(msg, pynmea2.types.talker.GGA):
                # Extract latitude and longitude
                latitude = msg.latitude
                longitude = msg.longitude
                return latitude, longitude
        except pynmea2.ParseError:
            # Handle parse error
            print("Could not parse GPS data.")
            return None, None

# Variables to store latitude and longitude
latitude, longitude = read_gps_data()

if __name__ == "__main__":
    # Print latitude and longitude for testing
    print("Latitude:", latitude, "Longitude:", longitude)
