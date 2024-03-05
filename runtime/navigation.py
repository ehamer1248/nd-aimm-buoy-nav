import os
import sys
import cv2
import depthai as dai
import numpy as np
from PIL import Image
from queue import Queue, Empty
import threading
from ultralytics import YOLO  
from calc import HostSpatialsCalc
from math import radians, sin, cos, sqrt, atan2, degrees
from engine import control_motors

########################
# Navitation Block
######################## 

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the Haversine distance between two points on the earth (specified in decimal degrees)
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    r = 6371  # Radius of Earth in kilometers. Use 3956 for miles
    return c * r * 1000  # Return distance in meters

def navigate_to_target(target_lat, target_lon):
    """
    Navigates from the current location to the target location by polling the current GPS coordinates
    and heading, and adjusting the heading accordingly.
    
    :param target_lat: Target latitude
    :param target_lon: Target longitude
    :returns: Navigation instruction ('left', 'right', or 'forward')
    """
    from gps import latitude, longitude  # Polls current GPS coordinates
    from heading import current_heading  # Polls current heading
    
    current_lat, current_lon = latitude(), longitude()  # Poll the current GPS coordinates
    current_head = current_heading()  # Poll the current heading
    
    # Calculate the distance to the target
    distance_to_target = haversine_distance(current_lat, current_lon, target_lat, target_lon)
    
    # Check if within 5-10ft (1.5-3 meters approximately) of the target
    if distance_to_target <= 3:
        return 'next_coordinate'  # Signal to move to the next coordinate

    def calculate_bearing(current_lat, current_lon, target_lat, target_lon):
        """
        Calculates the bearing from the current location to the target location.
        
        :returns: Bearing in degrees (0-360, where 0 is North)
        """
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(radians, [current_lat, current_lon, target_lat, target_lon])

        # Calculate the difference in longitude
        dlon = lon2 - lon1

        # Calculate the bearing
        x = sin(dlon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        bearing = atan2(x, y)

        # Convert bearing from radians to degrees
        bearing = degrees(bearing)
        bearing = (bearing + 360) % 360  # Normalize to 0-360

        return bearing

    def determine_turn_direction(current_heading, target_bearing):
        """
        Determines whether to turn left or right based on the current heading and target bearing.
        
        :returns: 'left', 'right', or 'forward'
        """
        # Calculate the difference between the current heading and the target bearing
        diff = target_bearing - current_heading

        # Normalize the difference to the range -180 to 180
        diff = (diff + 180) % 360 - 180

        # Determine the turn direction
        if abs(diff) < 10:  # If the difference is small, go forward
            return 'forward'
        elif diff > 0:
            return 'right'
        else:
            return 'left'

    # Calculate the bearing to the target
    target_bearing = calculate_bearing(current_lat, current_lon, target_lat, target_lon)

    # Determine the turn direction
    turn_direction = determine_turn_direction(current_head, target_bearing)

    # Return the navigation instruction
    return turn_direction
        
# State variables to track the last seen buoy
last_seen_buoy = {'type': None, 'position': None}  # type can be 'red' or 'green', position is (x, y) in camera coordinates

# def control_engines(turn_direction, degrees):
#     """
#     Control the boat's engines to maneuver around the buoy.
#     - turn_direction: 'left' or 'right', indicating the direction to turn after passing a buoy.
#     - degrees: The number of degrees to turn in the specified direction.
#     """
#     # Code to reverse both engines to stop
#     print("Reversing engines to stop.")
#     # Code to turn off one engine based on the turn_direction
#     if turn_direction == 'left':
#         print(f"Turning off right engine to turn {degrees} degrees left.")
#     else:  # turn_direction == 'right'
#         print(f"Turning off left engine to turn {degrees} degrees right.")
#     # Actual implementation to control engines goes here

# def adjust_course(buoy_position):
#     """
#     Adjust the boat's course based on the buoy's position.
#     - buoy_position: The position of the buoy in camera coordinates (x, y).
#     """
#     # Placeholder for course adjustment logic
#     # This could involve calculating the angle between the boat's current heading and the buoy,
#     # and adjusting the engines to correct the course.
#     print(f"Adjusting course based on buoy position: {buoy_position}")

# def process_result(result):
#     """
#     Process each result from the camera to determine the necessary action,
#     including tracking and adjusting course based on buoy position.
#     """
#     global last_seen_buoy
#     class_name = result['class_name']
#     bbox = result['bbox']  # Bounding box of the detected object
#     buoy_position = ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)  # Approximate center of the buoy

#     if 'green' in class_name or 'red' in class_name:
#         buoy_type = 'green' if 'green' in class_name else 'red'
#         if last_seen_buoy['type'] != buoy_type or last_seen_buoy['position'] != buoy_position:
#             last_seen_buoy = {'type': buoy_type, 'position': buoy_position}
#             print(f"{buoy_type.capitalize()} buoy detected at {buoy_position}, maneuvering.")
#             adjust_course(buoy_position)  # Adjust course based on current buoy position
                                
def results_consumer(results_queue, stop_event, target_lat, target_lon):
    while not stop_event.is_set():
        # Check for new results and process them if available
        try:
            result = results_queue.get(timeout=0.1)  # Adjust timeout as necessary
            print(result)  # This line prints the result; replace it with actual result processing logic
            # process_result(result)  # Process each result to track and adjust based on buoys
        except Empty:
            # No new results, but navigation will still be updated below
            pass

        # Update navigation instructions based on the latest GPS and heading
        turn_direction = navigate_to_target(target_lat, target_lon)
        
        # Pass the turn direction to control motors
        control_motors(turn_direction)

        # Add a short sleep to prevent this loop from consuming too much CPU
        time.sleep(0.1)
        