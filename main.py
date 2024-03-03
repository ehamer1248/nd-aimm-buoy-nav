import os
import sys
import cv2
import depthai as dai
import numpy as np
from PIL import Image
from queue import Queue, Empty
import threading
from ultralytics import YOLO  # Assuming this is a placeholder for the actual YOLO import
from calc import HostSpatialsCalc  # Make sure this is defined or imported correctly
from utility import *  # Import specific functions or classes instead of wildcard imports to avoid namespace pollution

########################
# Camera Block
######################## 

def createPipeline():
    pipeline = dai.Pipeline()

    # RGB Camera
    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setPreviewSize(300, 300)
    camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    camRgb.setInterleaved(False)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    camRgb.preview.link(xoutRgb.input)

    # Mono Cameras
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    
    stereo.initialConfig.setConfidenceThreshold(255)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)
    
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    # Depth Output
    xoutDepth = pipeline.create(dai.node.XLinkOut)
    xoutDepth.setStreamName("depth")
    stereo.depth.link(xoutDepth.input)

    # Disparity Output (for visualization)
    xoutDisp = pipeline.create(dai.node.XLinkOut)
    xoutDisp.setStreamName("disp")
    stereo.disparity.link(xoutDisp.input)

    return pipeline

def processCamera(deviceInfo, model_path, results_queue):
    model = YOLO(model_path)
    model.imgsz = 640
    
    text = TextHelper()


    with dai.Device(dai.OpenVINO.Version.VERSION_2021_4, deviceInfo, dai.UsbSpeed.SUPER) as device:
            print(f"===Connected to {deviceInfo.getMxId()}")

            pipeline = createPipeline()
            device.startPipeline(pipeline)

            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth")
            dispQueue = device.getOutputQueue(name="disp")

            hostSpatials = HostSpatialsCalc(device)
            hostSpatials.setDeltaRoi(4)

            frame_count = 0
            frame_skip = 5  # Process every 5th frame for inference

            while True:
                in_rgb = q_rgb.tryGet()
                depthData = depthQueue.tryGet()
                dispData = dispQueue.tryGet()

                # frame_count += 1
                # if frame_count % frame_skip != 0:
                #     continue  # Skip frame for throttling inference rate

                if in_rgb is not None and depthData is not None:
                    frame_rgb = in_rgb.getCvFrame()
                    pil_img = Image.fromarray(cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2RGB))
                    
                    results_list = model(pil_img, verbose=False)

                    if isinstance(results_list, list) and len(results_list) > 0:
                        results = results_list[0]
                        if results.boxes and len(results.boxes) > 0:
                            for box in results.boxes.data:
                                class_id = int(box[-1])
                                confidence = box[-2]
                                x1, y1, x2, y2 = map(int, box[:4])
                                class_name = results.names[class_id]
                                
                                # Scaling factors for each dimension
                                scale_factor_x = 640 / 300  # Horizontal scaling factor
                                scale_factor_y = 400 / 300   # Vertical scaling factor

                                # Scale coordinates down to match 400P resolution (640x400)
                                x1_scaled = int(x1 * scale_factor_x)
                                y1_scaled = int(y1 * scale_factor_y)
                                x2_scaled = int(x2 * scale_factor_x)
                                y2_scaled = int(y2 * scale_factor_y)
                                
                                # Pass a centroid
                                x_centroid = int((x1_scaled + x2_scaled) / 2)
                                y_centroid = int((y1_scaled + y2_scaled) / 2)    

                                # Calculate spatial coordinates for the entire bounding box
                                spatials, centroid = hostSpatials.calc_spatials(depthData, (x_centroid, y_centroid))
                                
                                result = {
                                    'device_id': deviceInfo.getMxId(),
                                    'class_name': class_name,
                                    'confidence': confidence,
                                    'bbox': [x1, y1, x2, y2],
                                    'spatials': spatials
                                }
                                 
                                results_queue.put(result)

                                # Check if spatial coordinates are valid before printing
                                # if not np.isnan(spatials['x']) and not np.isnan(spatials['y']) and not np.isnan(spatials['z']):
                                #     print(f"Detected on {deviceInfo.getMxId()}: {class_name} with confidence {confidence:.2f}")
                                #     print(f"Bounding box: x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}")
                                #     print(f"Spatial coordinates: X: {spatials['x']/1000:.2f}m, Y: {spatials['y']/1000:.2f}m, Z: {spatials['z']/1000:.2f}m")
                                # else:
                                #     print(f"Detected on {deviceInfo.getMxId()}: {class_name} with confidence {confidence:.2f}")
                                #     print("Spatial coordinates: Not available (NaN)"
                                
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
        
########################
# Main Block
######################## 

def main():
    
    results_queue = Queue()
    stop_event = threading.Event()
    
    min_devices_required = 2
    retry_delay = 5
    max_retries = 10
    deviceInfos = []  # Placeholder for device information retrieval
    
    # This part remains unchanged
    for attempt in range(max_retries):
        deviceInfos = dai.Device.getAllAvailableDevices()
        if len(deviceInfos) >= min_devices_required:
            print(f"Found {len(deviceInfos)} devices.")
            break  # Required number of devices found, exit the loop
        else:
            print(f"Less than {min_devices_required} devices found, retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)

    if len(deviceInfos) < min_devices_required:
        print(f"Less than {min_devices_required} devices found after retrying, exiting.")
        sys.exit(0)

    model_paths = ['/home/ndaimm/ndaimm/onhost/custom1.pt', '/home/ndaimm/ndaimm/onhost/custom2.pt']
    threads = []

    for i, deviceInfo in enumerate(deviceInfos):
        model_path = model_paths[i % len(model_paths)]
        # Pass display_queue as an argument to processCamera
        t = threading.Thread(target=processCamera, args=(deviceInfo, model_path, results_queue))
        t.start()
        threads.append(t)
        
    consumer_thread = threading.Thread(target=results_consumer, args=(results_queue, stop_event))
    consumer_thread.start()

    for t in threads:
        t.join()  # Ensure all threads have finished before exiting
        
    stop_event.set()
    consumer_thread.join()

if __name__ == "__main__":
    main()