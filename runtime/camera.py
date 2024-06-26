#!/usr/bin/env python3

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

    with dai.Device(dai.OpenVINO.Version.VERSION_2021_4, deviceInfo, dai.UsbSpeed.SUPER) as device:
        print(f"===Connected to {deviceInfo.getMxId()}")

        pipeline = createPipeline()
        device.startPipeline(pipeline)

        q_rgb = device.getOutputQueue(name="rgb", maxSize=10, blocking=False)
        depthQueue = device.getOutputQueue(name="depth", maxSize=10, blocking=False)
        dispQueue = device.getOutputQueue(name="disp", maxSize=10, blocking=False)

        hostSpatials = HostSpatialsCalc(device)
        hostSpatials.setDeltaRoi(4)

        frame_count = 0
        frame_skip = 3  # Process every 3rd frame

        while True:
            in_rgb = q_rgb.get()
            depthData = depthQueue.get()

            frame_count += 1
            if frame_count % frame_skip != 0:
                continue  # Skip frame for throttling inference rate

            if in_rgb is not None and depthData is not None:
                frame_rgb = in_rgb.getCvFrame()
                pil_img = Image.fromarray(cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2RGB))

                #print(f"Processing frame on {deviceInfo.getMxId()}")  # Debug: Print when a frame is being processed

                results_list = model(pil_img, verbose=False)

                if isinstance(results_list, list) and len(results_list) > 0:
                    results = results_list[0]
                    #print(f"Detections found on {deviceInfo.getMxId()}: {len(results.boxes)}")  # Debug: Print the number of detections

                    if results.boxes and len(results.boxes) > 0:
                        for box in results.boxes.data:
                            class_id = int(box[-1])
                            confidence = box[-2]
                            x1, y1, x2, y2 = map(int, box[:4])
                            class_name = results.names[class_id]

                            # Scaling factors for each dimension
                            scale_factor_x = 640 / 300  # Horizontal scaling factor
                            scale_factor_y = 400 / 300  # Vertical scaling factor

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
                            #print(f"Result added to queue on {deviceInfo.getMxId()}: {result}")  # Debug: Print the result added to the queue
                else:
                    print(f"No detections found on {deviceInfo.getMxId()}")  # Debug: Print when no detections are found
            else:
                print(f"Skipping frame on {deviceInfo.getMxId()} due to missing data")  # Debug: Print when frame is skipped due to missing data