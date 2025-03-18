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
from path_planning import main as path_planning
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
from user_input import main as user_input

# Used to enable safe keyboard triggered shutdown
global KILL_THREAD
KILL_THREAD = False

def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

def show_image(lane_detect_queue: multiprocessing.Queue,image_queue: multiprocessing.Queue):
   print("Showing Images")
   while True: 
        img_display = image_queue.get()
        lane_display = lane_detect_queue.get()
        cv2.imshow("Lane Detection", lane_display)
        cv2.imshow("YOLOv8 Detection", img_display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
   cv2.destroyAllWindows()


if __name__ == "__main__":
    # Create queues for communication between processes
    perception_queue = multiprocessing.Queue()
    image_queue = multiprocessing.Queue()
    lane_detect_queue = multiprocessing.Queue()
    command_queue = multiprocessing.Queue()
    PU_DO_queue = multiprocessing.Queue()
    path_queue = multiprocessing.Queue()
    # Start the environment setup process
    environment_process = multiprocessing.Process(target=setup)
    environment_process.start()
    environment_process.join()

    command_queue.put("stop")

    # Start the perception process
    perception_process = multiprocessing.Process(
        target=perception, args=(perception_queue,image_queue,lane_detect_queue)
    )
    perception_process.start()
    #obtain pick-up and drop-off locations from user
    pickup_waypoint = int(input("Please enter the pickup location (waypoint): "))
    PU_DO_queue.put(pickup_waypoint)
    dropoff_waypoint = int(input("Please enter the drop-off location (waypoint): "))
    PU_DO_queue.put(dropoff_waypoint)
    print(f"Received Pick-up Waypoint: {pickup_waypoint}")
    print(f"Received Drop-Off Waypoint: {dropoff_waypoint}")
    # Start the path planning process
    pathplanning_process = multiprocessing.Process(
        target=path_planning, args=(path_queue,PU_DO_queue)
    )
    
    #plan path based on user input
    pathplanning_process.start() 
     
    pid_controller_process = multiprocessing.Process(
       target=PID, args=(command_queue,path_queue,PU_DO_queue)
    )
        
    pid_controller_process.start()

    #show_image(lane_detect_queue, image_queue)

    # perception_process.join()
    #pathplanning_process.join()  
   
    # pid_controller_process.join()

 


