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
MIN_BB_HEIGHT = 64
MIN_BB_WIDTH = 64
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
            width = float(results.boxes.xywh[0, 2])
            classLabel = int(results.boxes.cls[0])
            if classLabel == 1:
                height = float(results.boxes.xywh[0, 3])
                if height > MIN_BB_HEIGHT:
                    command_queue.put("stop")
                    if(classLabel == 2):    
                        print("Traffic Light Green Detected")       
                    if(classLabel == 3):        
                        print("Traffic Light Red Detected")    
                    if(classLabel == 4):
                        print("Traffic Light Yellow Detected")
                    if(classLabel == 5):
                        print("Yield Sign Detected")
        
        perception_queue.put(results)
        lane_detect_queue.put(binaryImage)
        image_queue.put(processedImg)

if __name__ == "__main__":
    percpetion_queue = multiprocessing.Queue()
    image_queue = multiprocessing.Queue()
    lane_detect_queue = multiprocessing.Queue()
    main(percpetion_queue, image_queue, lane_detect_queue)  # Call the main function directly