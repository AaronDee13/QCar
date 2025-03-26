import multiprocessing
import os
from pathlib import Path
import multiprocessing
import time

from ultralytics import YOLO
import cv2
from qvl.qlabs import QuanserInteractiveLabs
from qvl.free_camera import QLabsFreeCamera
from qvl.qcar import QLabsQCar
from qvl.real_time import QLabsRealTime
from PIL import Image
import pal.resources.rtmodels as rtmodels
from pal.products.qcar import QCar,QCarCameras
from pal.utilities.math import *
from hal.utilities.image_processing import ImageProcessing
import numpy as np
import math

CAMERA = QLabsQCar.CAMERA_RGB
model_path = Path("models/best.pt")
MIN_BB_STOP_SIZE = 50
MIN_BB_YIELD_SIZE = 30
MIN_BB_TRAFFIC_LIGHT_SIZE = 30

STOP_SIGN_WAIT_TIME = 4
YIELD_SIGN_SLOW_TIME = 5
steeringFilter = Filter().low_pass_first_order_variable(25,0.033)
next(steeringFilter)

def setup_qcar(
    initialPosition=[-1.205, -0.83, 0.005],
    initialOrientation=[0, 0, -44.7],
    rtModel=rtmodels.QCAR,
):
    # connect to Qlabs
    qlabs = QuanserInteractiveLabs()
    try:
        qlabs.open("localhost")
        print("QCar Perception Started")
    except:
        print("Unable to connect to QLabs")
        quit()

    # Delete any previous QCar instances and stop any running spawn models
    # qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    # Spawn a QCar at the given initial pose
    qcar = QLabsQCar(qlabs)
    qcar.spawn_id(
        actorNumber=0,
        location=[p * 10 for p in initialPosition],
        rotation=initialOrientation,
        waitForConfirmation=True,
    )

    # Create a new camera view and attach it to the QCar
    hcamera = QLabsFreeCamera(qlabs)
    hcamera.spawn()
    qcar.possess()

    # Start spawn model
    QLabsRealTime().start_real_time_model(rtModel)

    return qcar

def main(perception_queue: multiprocessing.Queue, image_queue: multiprocessing.Queue, 
         lane_detect_queue: multiprocessing.Queue, command_queue: multiprocessing.Queue):
    model = YOLO(model_path)
    car = setup_qcar()
    dt = 0.033
    lastInteractedWithStopSign = 0
    lastInteractedWithYieldSign = 0
    lastSawRedLight = 0
    waitingAtStopSign = False
    waitingAtYieldSign = False
    waitingAtTrafficLight = False
    while True:
        # start = time.time()
        image = car.get_image(CAMERA)[1]
        results = model.predict(image, verbose=False)[0]
        
        #classLabel = print(int(results.boxes.cls[0]))
        #print(results.names)
        #print(results.boxes.xyxy[0])
        # width = print(float(results.boxes.xywh[0, 2]))
        # height = print(float(results.boxes.xywh[0, 3]))
        processedImg = results.plot(
            img=image,  # Plotting on the original image
            conf=True,  # Display confidence score
            boxes=True,  # Draw bounding boxes
            labels=True,  # Display labels
            masks=False,  # Assuming no masks to plot
            probs=False,  # Show probabilities if desired
            line_width=2,  # Line width of bounding boxes
            font_size=None,  # Automatically scale font size
            pil=False,  # Return as a numpy array
        )
        # Convert to HSV and then threshold it for yellow
        hsvBuf = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        binaryImage = ImageProcessing.binary_thresholding(frame= hsvBuf,
											lowerBounds=np.array([10, 0, 0]),
											upperBounds=np.array([45, 255, 255]))

        if len(results.boxes.cls) > 0:
            #we need to find the nearest class label, so the largest
            #classLabel = int(results.boxes.cls[0])
            #height = float(results.boxes.xywh[0, 3])
            height = 0
            classLabel = 0
            for i in range(len(results.boxes.cls)):
                if float(results.boxes.xywh[i, 3]) > height:
                    tempClassLabel = int(results.boxes.cls[i])
                    #fun BS, we detect traffic lights on the left and right too, so we need to really just filter for the center of the screen
                    if (tempClassLabel == 2 or tempClassLabel == 3 or tempClassLabel == 4):
                        #check if any of the BB is outside of the window
                        if float(results.boxes.xyxyn[i, 0]) < 0 or float(results.boxes.xyxyn[i, 2]) > 1:
                            continue
                        if float(results.boxes.xyxyn[i, 1]) < 0 or float(results.boxes.xyxyn[i, 3]) > 1:
                            continue
                        #get horizontal window width
                        horizontalCenterNormalized = (float(results.boxes.xyxyn[i, 0]) + float(results.boxes.xyxyn[i, 2])) / 2
                        if horizontalCenterNormalized - 0.5 < TRAFFIC_LIGHT_CENTERED_WIDTH and horizontalCenterNormalized - 0.5 > -TRAFFIC_LIGHT_CENTERED_WIDTH:
                            height = float(results.boxes.xywh[i, 3])
                            classLabel = int(results.boxes.cls[i])
                            #continue to next
                            continue
                        #skip if it isnt
                        continue
                    height = float(results.boxes.xywh[i, 3])
                    classLabel = int(results.boxes.cls[i])
            
            GRACE_PERIOD = 1
            TRAFFIC_LIGHT_CENTERED_WIDTH = 0.5

            #print("Class Label: ", classLabel, "Height: ", height)
            if classLabel == 1: #Stop Sign
                if height > MIN_BB_STOP_SIZE:
                    # grace period check
                    if time.time() - lastInteractedWithStopSign > STOP_SIGN_WAIT_TIME + GRACE_PERIOD:
                        lastInteractedWithStopSign = time.time()
                        waitingAtStopSign = True
                        command_queue.put("stop")
                        print("Stop Sign Detected")
            if(classLabel == 5): #Yield Sign
                if height > MIN_BB_YIELD_SIZE:
                    if time.time() - lastInteractedWithYieldSign > YIELD_SIGN_SLOW_TIME + GRACE_PERIOD:
                        lastInteractedWithYieldSign = time.time()
                        waitingAtYieldSign = True
                        command_queue.put("slow")
                        print("Yield Sign Detected")
            if(classLabel == 2):
                if height > MIN_BB_TRAFFIC_LIGHT_SIZE:
                    print("Traffic Light Green Detected")
                    command_queue.put("normal")
            if(classLabel == 3 or classLabel == 4): #red or yellow
                if height > MIN_BB_TRAFFIC_LIGHT_SIZE:
                    lastSawRedLight = time.time()
                    waitingAtTrafficLight = True
                    command_queue.put("stop")
            

        # determine if we should move forward after waiting the appropriate time
        if time.time() - lastInteractedWithStopSign > STOP_SIGN_WAIT_TIME and waitingAtStopSign:
            waitingAtStopSign = False
            lastInteractedWithStopSign = time.time() - STOP_SIGN_WAIT_TIME
            command_queue.put("normal")
        
        if time.time() - lastInteractedWithYieldSign > YIELD_SIGN_SLOW_TIME and waitingAtYieldSign:
            waitingAtYieldSign = False
            lastInteractedWithYieldSign = time.time() - YIELD_SIGN_SLOW_TIME
            command_queue.put("normal")

        TRAFFIC_LIGHT_TIMEOUT = 2

        if time.time() - lastSawRedLight > TRAFFIC_LIGHT_TIMEOUT and waitingAtTrafficLight:
            waitingAtTrafficLight = False
            lastSawRedLight = time.time() - TRAFFIC_LIGHT_TIMEOUT
            command_queue.put("normal")
        
        perception_queue.put(results)
        lane_detect_queue.put(binaryImage)
        image_queue.put(processedImg)

if __name__ == "__main__":
    percpetion_queue = multiprocessing.Queue()
    image_queue = multiprocessing.Queue()
    lane_detect_queue = multiprocessing.Queue()
    main(percpetion_queue, image_queue, lane_detect_queue)  # Call the main function directly
