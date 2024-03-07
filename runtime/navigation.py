import os
import sys
import cv2
import depthai as dai
import numpy as np
from PIL import Image
from queue import Queue, Empty
import threading
import time
from ultralytics import YOLO  
from calc import HostSpatialsCalc
from math import radians, sin, cos, sqrt, atan2, degrees
from engine import control_motors
from gps import read_gps_data_ublox

########################
# Navitation Block
######################## 

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the Haversine distance between two points on the earth (specified in decimal degrees).
    This version uses the mean Earth radius for slightly improved accuracy.

    :param lat1: Latitude of the first point in decimal degrees
    :param lon1: Longitude of the first point in decimal degrees
    :param lat2: Latitude of the second point in decimal degrees
    :param lon2: Longitude of the second point in decimal degrees
    :return: Distance between the two points in meters
    """
    # Mean radius of Earth in kilometers, accounting for Earth's ellipsoidal shape
    r = 6371.0088

    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Calculate differences in coordinates
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Haversine formula
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    # Distance in meters
    distance = c * r * 1000

    return distance

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
    Determines whether to turn left or right based on the current heading and target bearing, considering the shortest path.

    :returns: 'left', 'right', or 'forward'
    """
    # Calculate the difference between the current heading and the target bearing
    diff = target_bearing - current_heading

    # Normalize the difference to the range 0 to 360
    diff = diff % 360

    # Determine the turn direction based on the shortest path
    if abs(diff) < 10 or abs(diff) > 350:  # If the difference is small, go forward
        return 'forward'
    elif diff > 180:
        return 'left'
    else:
        return 'right'
    
########################
# Movement Call Block
######################## 

def navigate_to_target(target_lat, target_lon, pwm1, pwm2):
    """
    Navigates from the current location to the target location by polling the current GPS coordinates
    and heading, and adjusting the heading accordingly.

    :param target_lat: Target latitude
    :param target_lon: Target longitude
    :returns: Navigation instruction ('left', 'right', or 'forward')
    """
    current_lat, current_lon, current_head, current_accHeading = read_gps_data_ublox()  # Poll the current GPS coordinates
    
    # Calculate the distance to the target
    distance_to_target = haversine_distance(current_lat, current_lon, target_lat, target_lon)

    target_coordinate_list

    # Check if within 5-10ft (1.5-3 meters approximately) of the target
    if distance_to_target <= 3:
        control_motors(pwm1, pwm2, 'stop')
        return 'next_coordinate'  # Signal to move to the next coordinate

    # Calculate the bearing to the target
    target_bearing = calculate_bearing(current_lat, current_lon, target_lat, target_lon)

    # Determine the turn direction
    turn_direction = determine_turn_direction(current_head, target_bearing)

    # Return the navigation instruction
    return turn_direction
























        
# def adjust_course(buoy_position, buoy_color):
#     """
#     Adjust the boat's course based on the buoy's position.
#     - buoy_position: The position of the buoy in camera coordinates (x, y).

#      print(f"Adjusting course based on {buoy_color} buoy position: {buoy_position}")

##########
# I do not know what the below does anymore.
##########

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

# State variables to track the last seen buoy
last_seen_buoy = {'type': None, 'position': None}  # type can be 'red' or 'green', position is (x, y) in camera coordinates
                                
        
