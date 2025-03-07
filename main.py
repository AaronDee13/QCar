import time
import cv2
import multiprocessing
from environment_setup import main as setup
from PID_Controller import main as PID
#from EnvironmentSetup import main as environment
#from perception import main as perception_main
#from controller import main as controller_main
#from PID_Controller import main as pid_controller


def display_images(image_data):
    while True:
        img_display = image_data.result()
        cv2.imshow("YOLOv8 Detection", img_display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    command_queue = multiprocessing.Queue()
    environment_process = multiprocessing.Process(target=setup)


    pid_controller_process = multiprocessing.Process(
        target=PID, args=(command_queue,)
    )

    #environment_process.start()
    time.sleep(2)
    pid_controller_process.start()

    #environment_process.join()
    pid_controller_process.join()