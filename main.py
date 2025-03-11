import time
import cv2
import multiprocessing
import os
from environment_setup import setup as setup
from PID_Controller import main as PID
from pal.products.qcar import QCarRealSense
#from EnvironmentSetup import main as environment
from perception import main as perception
#from controller import main as controller_main
#from PID_Controller import main as pid_controller

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


if __name__ == "__main__":

    perception_queue = multiprocessing.Queue()
    image_queue = multiprocessing.Queue()
    command_queue = multiprocessing.Queue()
    environment_process = multiprocessing.Process(target=setup)

    perception_process = multiprocessing.Process(
        target=perception, args=(perception_queue,)
    )

    pid_controller_process = multiprocessing.Process(
       target=PID, args=(command_queue,)
    )

    environment_process.start()
    time.sleep(2)
    perception_process.start()
    time.sleep(2)
    pid_controller_process.start()
    time.sleep(2)

    environment_process.join()
    perception_process.join()
    pid_controller_process.join()

