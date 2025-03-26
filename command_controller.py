import multiprocessing
import time



STOP_SIGN_MINIMUM_HEIGHT = 64
RED_LIGHT_MINIMUM_WIDTH = 22
MIN_BB_HEIGHT = 64
STOP_SIGN_DURATION = 3
STOP_SIGN = 1
YIELD_SIGN = 5

def main(perception_queue: multiprocessing.Queue, command_queue: multiprocessing.Queue):
    while True:
        if not perception_queue.empty():
            results = perception_queue.get()
            if len(results.boxes.cls) > 0:
                cls = int(results.boxes.cls[0])
                if cls == STOP_SIGN:
                    height = float(results.boxes.xywh[0, 3])
                    
                    if height > STOP_SIGN_MINIMUM_HEIGHT:
                        command_queue.put("stop")
                        print('Stop command sent')
                        # wait for duration
                        t1 = time.time()
                        while time.time() - t1 <= STOP_SIGN_DURATION:
                            if not perception_queue.empty():
                                # consume unneeded observations
                                perception_queue.get()
                        command_queue.put("go")

                        while True:
                            if not perception_queue.empty():
                                results = perception_queue.get()
                                # verify if there are any results to check
                                if len(results.boxes.cls) > 0:
                                    cls = int(results.boxes.cls[0])
                                    height = height = float(results.boxes.xywh[0, 3])
                                    if(
                                        cls != STOP_SIGN
                                        or cls == STOP_SIGN
                                        and height
                                        <= STOP_SIGN_MINIMUM_HEIGHT
                                        - STOP_SIGN_MINIMUM_HEIGHT * 0.1
                                    ):
                                        break
                if cls == YIELD_SIGN:
                    command_queue.put("stop")
                    print('Yield command sent')
                    time.sleep(2)
                    command_queue.put("go")
                    print('Go command sent')