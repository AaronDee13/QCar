import time
import cv2
import multiprocessing
import os
import signal
from environment_setup import setup as setup
from PID_Controller import main as PID
from pal.products.qcar import QCarRealSense
#from EnvironmentSetup import main as environment
from perception import main as perception
#from controller import main as controller_main
#from PID_Controller import main as pid_controller
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar import QLabsQCar
from qvl.free_camera import QLabsFreeCamera
from qvl.traffic_light import QLabsTrafficLight
from qvl.real_time import QLabsRealTime
from qvl.basic_shape import QLabsBasicShape
from qvl.system import QLabsSystem
from qvl.walls import QLabsWalls
from qvl.flooring import QLabsFlooring
from qvl.stop_sign import QLabsStopSign
from qvl.crosswalk import QLabsCrosswalk
import pal.resources.rtmodels as rtmodels


# Used to enable safe keyboard triggered shutdown
global KILL_THREAD
KILL_THREAD = False

def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

def show_image(image_queue: multiprocessing.Queue):
    while True:
        img_display = image_queue.get()
        cv2.imshow("YOLOv8 Detection", img_display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()

def show_lane(lane_detect_queue: multiprocessing.Queue):
    while True:
        img_display = lane_detect_queue.get()
        cv2.imshow("Lane Detection", img_display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":

    # Create queues for communication between processes
    perception_queue = multiprocessing.Queue()
    image_queue = multiprocessing.Queue()
    lane_detect_queue = multiprocessing.Queue()
    command_queue = multiprocessing.Queue()

    command_queue.put("stop")
    # Start the environment setup process
    environment_process = multiprocessing.Process(target=setup)
    environment_process.start()
    environment_process.join()

    # Start the perception process
    perception_process = multiprocessing.Process(
        target=perception, args=(perception_queue,image_queue,lane_detect_queue)
    )
    perception_process.start()
    show_lane(lane_detect_queue)
    show_image(image_queue)

    pid_controller_process = multiprocessing.Process(
       target=PID, args=(command_queue,)
    )

    pid_controller_process.start()

    perception_process.join()
    pid_controller_process.join()
