import os
import math
import time
import multiprocessing
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
from qvl.yield_sign import QLabsYieldSign
from qvl.crosswalk import QLabsCrosswalk
import pal.resources.rtmodels as rtmodels

def setup(
    initialPosition=[-1.205, -0.83, 0.005],
    initialOrientation=[0, 0, -44.7],
    rtModel=rtmodels.QCAR,
):
    # connect to Qlabs
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    try:
        qlabs.open("localhost")
        print("Connected to QLabs")
        print("Setting Up Environment...")
    except:
        print("Unable to connect to QLabs")
        quit()

    # Delete any previous QCar instances and stop any running spawn models
    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    # Spawn stop signs

    StopSign1 = QLabsStopSign(qlabs)
    StopSign1.spawn(
        location=[17.85, 17, 0.0],
        rotation=[0.0, 0.0, 1.55],
        waitForConfirmation=False,
    )

    StopSign2 = QLabsStopSign(qlabs)
    StopSign2.spawn(
        location=[24.328, 2.5, 0.0],
        rotation=[0.0, 0.0, -1.6],
        waitForConfirmation=False,
    )

    StopSign5 = QLabsStopSign(qlabs)
    StopSign5.spawn(
        location=[0, -12.521, 0.2],
        rotation=[0.0, 0.0, 3.14],
        waitForConfirmation=False,
    )

    StopSign6 = QLabsStopSign(qlabs)
    StopSign6.spawn(
        location=[13.85, 6.3, 0],
        rotation=[0.0, 0.0, 3.14],
        waitForConfirmation=False,
    )

    YieldSign = QLabsYieldSign(qlabs)
    YieldSign.spawn(
        location=[24.271, 32.997, 0.18],
        rotation=[0.0, 0.0, -1.5],
        waitForConfirmation=False,
    )

    YieldSign2 = QLabsYieldSign(qlabs)
    YieldSign2.spawn(
        location=[12.5, 29.5, 0.0],
        rotation=[0.0, 0.0, 3.5],
        waitForConfirmation=False,
    )

    YieldSign3 = QLabsYieldSign(qlabs)
    YieldSign3.spawn(
        location=[6.2, 39, 0.0],
        rotation=[0.0, 0.0, 2.3],
        waitForConfirmation=False,
    )

    # spawn traffic lights
    #these two are across from eachother
    TrafficLight1 = QLabsTrafficLight(qlabs)
    TrafficLight1.spawn(location=[4.78, 15.5, 0.0], rotation=[0.0, 0.0, 0.0])
    TrafficLight1.set_color(QLabsTrafficLight.COLOR_RED)

    TrafficLight5 = QLabsTrafficLight(qlabs)
    TrafficLight5.spawn(location=[-2.1, 3.57, 0.0], rotation=[0.0, 0.0, 3.14])
    TrafficLight5.set_color(QLabsTrafficLight.COLOR_RED)

    #these two are across from eachother
    TrafficLight4 = QLabsTrafficLight(qlabs)
    TrafficLight4.spawn(location=[-4.85, 13.3, 0.0], rotation=[0.0, 0.0, 1.57])
    TrafficLight4.set_color(QLabsTrafficLight.COLOR_RED)
 
    TrafficLight6 = QLabsTrafficLight(qlabs)
    TrafficLight6.spawn(location=[7.34, 6.3, 0.0], rotation=[0.0, 0.0, 4.71])
    TrafficLight6.set_color(QLabsTrafficLight.COLOR_RED)
    
    while True:

        TrafficLight1.set_color(QLabsTrafficLight.COLOR_GREEN)
        TrafficLight5.set_color(QLabsTrafficLight.COLOR_GREEN)
        TrafficLight4.set_color(QLabsTrafficLight.COLOR_RED)
        TrafficLight6.set_color(QLabsTrafficLight.COLOR_RED)

        t0 = time.time()
        while time.time() - t0 < 10:
            pass
        
        TrafficLight1.set_color(QLabsTrafficLight.COLOR_YELLOW)
        TrafficLight5.set_color(QLabsTrafficLight.COLOR_YELLOW)

        t0 = time.time()
        while time.time() - t0 < 3:
            pass

        TrafficLight1.set_color(QLabsTrafficLight.COLOR_RED)
        TrafficLight5.set_color(QLabsTrafficLight.COLOR_RED)
        TrafficLight4.set_color(QLabsTrafficLight.COLOR_GREEN)
        TrafficLight6.set_color(QLabsTrafficLight.COLOR_GREEN)

        t0 = time.time()
        while time.time() - t0 < 10:
            pass

        TrafficLight4.set_color(QLabsTrafficLight.COLOR_YELLOW)
        TrafficLight6.set_color(QLabsTrafficLight.COLOR_YELLOW)


        t0 = time.time()
        while time.time() - t0 < 3:
            pass
        
    return 0


def terminate():
    QLabsRealTime().terminate_real_time_model(rtmodels.QCAR_STUDIO)


if __name__ == "__main__":
    setup()
