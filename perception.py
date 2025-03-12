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


CAMERA = QLabsQCar.CAMERA_RGB
model_path = Path("models/best.pt")

def setup_qcar(
    initialPosition=[-1.205, -0.83, 0.005],
    initialOrientation=[0, 0, -44.7],
    rtModel=rtmodels.QCAR,
):
    # connect to Qlabs
    os.system("cls")
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    try:
        qlabs.open("localhost")
        print("Connected to QLabs")
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

def main(perception_queue: multiprocessing.Queue):
    model = YOLO(model_path)
    car = setup_qcar()

    while True:
        image = car.get_image(CAMERA)[1]
        results = model.predict(image, verbose=False)[0]
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
        
        perception_queue.put(results)

        cv2.imshow("YOLOv8 Detection", processedImg)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
