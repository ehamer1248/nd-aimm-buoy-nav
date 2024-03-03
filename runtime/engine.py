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

########################
# Engine Block
######################## 

# State variables to track the last seen buoy
last_seen_buoy = {'type': None, 'position': None}  # type can be 'red' or 'green', position is (x, y) in camera coordinates

def control_engines(turn_direction, degrees):
    """
    Control the boat's engines to maneuver around the buoy.
    - turn_direction: 'left' or 'right', indicating the direction to turn after passing a buoy.
    - degrees: The number of degrees to turn in the specified direction.
    """
    # Code to reverse both engines to stop
    print("Reversing engines to stop.")
    # Code to turn off one engine based on the turn_direction
    if turn_direction == 'left':
        print(f"Turning off right engine to turn {degrees} degrees left.")
    else:  # turn_direction == 'right'
        print(f"Turning off left engine to turn {degrees} degrees right.")
    # Actual implementation to control engines goes here

def adjust_course(buoy_position):
    """
    Adjust the boat's course based on the buoy's position.
    - buoy_position: The position of the buoy in camera coordinates (x, y).
    """
    # Placeholder for course adjustment logic
    # This could involve calculating the angle between the boat's current heading and the buoy,
    # and adjusting the engines to correct the course.
    print(f"Adjusting course based on buoy position: {buoy_position}")

def process_result(result):
    """
    Process each result from the camera to determine the necessary action,
    including tracking and adjusting course based on buoy position.
    """
    global last_seen_buoy
    class_name = result['class_name']
    bbox = result['bbox']  # Bounding box of the detected object
    buoy_position = ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)  # Approximate center of the buoy

    if 'green' in class_name or 'red' in class_name:
        buoy_type = 'green' if 'green' in class_name else 'red'
        if last_seen_buoy['type'] != buoy_type or last_seen_buoy['position'] != buoy_position:
            last_seen_buoy = {'type': buoy_type, 'position': buoy_position}
            print(f"{buoy_type.capitalize()} buoy detected at {buoy_position}, maneuvering.")
            adjust_course(buoy_position)  # Adjust course based on current buoy position
                                
def results_consumer(results_queue, stop_event):
    while not stop_event.is_set() or not results_queue.empty():
        try:
            result = results_queue.get(timeout=0.1)  # Adjust timeout as necessary
            print(result)  # Replace this with your actual result processing logic
            process_result(result)  # Process each result to track and adjust based on buoys
        except Empty:
            continue
        