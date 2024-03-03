import os
import sys
import cv2
import depthai as dai
import numpy as np
from PIL import Image
import threading
from ultralytics import YOLO  # Assuming this is a placeholder for the actual YOLO import
from calc import HostSpatialsCalc  # Make sure this is defined or imported correctly
from utility import *  # Import specific functions or classes instead of wildcard imports to avoid namespace pollution

MAX_DISPARITY = 96

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

def processCamera(deviceInfo, model_path):
    cv2.namedWindow(f"disp image")
    model = YOLO(model_path)
    model.imgsz = 640
    
    text = TextHelper()


    with dai.Device(dai.OpenVINO.Version.VERSION_2021_4, deviceInfo, dai.UsbSpeed.SUPER) as device:
        print(f"===Connected to {deviceInfo.getMxId()}")

        pipeline = createPipeline()
        device.startPipeline(pipeline)
        stereo = pipeline.create(dai.node.StereoDepth)


        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        depthQueue = device.getOutputQueue(name="depth")
        dispQueue = device.getOutputQueue(name="disp")

        hostSpatials = HostSpatialsCalc(device)
        hostSpatials.setDeltaRoi(2)  # Set delta ROI to a small value to focus on the center of the bounding box

        while True:
            in_rgb = q_rgb.tryGet()
            depthData = depthQueue.tryGet()
            #dispData = dispQueue.tryGet()

            if in_rgb is not None and depthData is not None:
                frame_rgb = in_rgb.getCvFrame()
                pil_img = Image.fromarray(cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2RGB))
                
                # Perform inference
                results_list = model(pil_img, verbose=False)

                # Check for results and print them
                if isinstance(results_list, list) and len(results_list) > 0:
                    results = results_list[0]  # Access the first Results object
                    if results.boxes and len(results.boxes) > 0:
                        for box in results.boxes.data:
                            class_id = int(box[-1])
                            confidence = box[-2]
                            x1, y1, x2, y2 = map(int, box[:4])
                            class_name = results.names[class_id]
                            
                            y = 200
                            x = 300
                            
                            # Scaling factors for each dimension
                            scale_factor_x = 640 / 1280  # Horizontal scaling factor
                            scale_factor_y = 400 / 800   # Vertical scaling factor

                            # Scale coordinates down to match 400P resolution (640x400)
                            x1_scaled = int(x1 * scale_factor_x)
                            y1_scaled = int(y1 * scale_factor_y)
                            x2_scaled = int(x2 * scale_factor_x)
                            y2_scaled = int(y2 * scale_factor_y)
                            
                            # Pass a centroid
                            
                            x_centroid = int((x1_scaled + x2_scaled) / 2)
                            y_centroid = int((y1_scaled + y2_scaled) / 2)

                            # Calculate spatial coordinates for the entire bounding box
                            spatials, centroid = hostSpatials.calc_spatials(depthData, (x_centroid,y_centroid))

                            # Check if spatial coordinates are valid before printing
                            if not np.isnan(spatials['x']) and not np.isnan(spatials['y']) and not np.isnan(spatials['z']):
                                print(f"Detected on {deviceInfo.getMxId()}: {class_name} with confidence {confidence:.2f}")
                                print(f"Bounding box: x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}")
                                print(f"Spatial coordinates: X: {spatials['x']/1000:.2f}m, Y: {spatials['y']/1000:.2f}m, Z: {spatials['z']/1000:.2f}m")
                            else:
                                print(f"Detected on {deviceInfo.getMxId()}: {class_name} with confidence {confidence:.2f}")
                                print("Spatial coordinates: Not available (NaN)")

                            dispData = dispQueue.get()
                            if dispData is not None:
                                print('Showing image...')
                                dispImg = dispData.getFrame()
                                cv2.rectangle(dispImg, (x1_scaled, y1_scaled), (x2_scaled, y2_scaled), (255, 255, 255), 2)
                                cv2.imshow(f'disp image', dispImg)
                            else:
                                print('No image')

min_devices_required = 2
retry_delay = 5  # seconds
max_retries = 10  # you can adjust this number based on your requirements

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

processCamera(deviceInfos[0], model_paths[0])
'''
for i, deviceInfo in enumerate(deviceInfos):
    if i == 0: continue

    model_path = model_paths[i % len(model_paths)]
    t = threading.Thread(target=processCamera, args=(deviceInfo, model_path))
    processCamera(deviceInfo, model_path)
    t.start()
    threads.append(t)

for t in threads:
    t.join()
'''
