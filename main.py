import time
import cv2
from concurrent.futures import ProcessPoolExecutor

#from EnvironmentSetup import main as environment
#from perception import main as perception_main
#from controller import main as controller_main
from PID_Controller import main as pid_controller


def display_images(image_data):
    while True:
        img_display = image_data.result()
        cv2.imshow("YOLOv8 Detection", img_display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    with ProcessPoolExecutor() as executor:
        #environment_future = executor.submit(environment)
        time.sleep(2)
      #  perception_future = executor.submit(perception_main)
       # controller_future = executor.submit(controller_main)
        time.sleep(4)
        pid_controller_future = executor.submit(pid_controller)

        #display_images(perception_future)

        #environment_future.result()
        #perception_future.result()
        #controller_future.result()
        pid_controller_future.result()