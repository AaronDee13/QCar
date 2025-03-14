import os
import math

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
    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    # Spawn stop signs
    
    # StopSign1 = QLabsStopSign(qlabs)
    # StopSign1.spawn(
    #     location=[24.328, 18.0, 0.0],
    #     rotation=[0.0, 0.0, -1.6],
    #     waitForConfirmation=False,
    # )

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

    # StopSign3 = QLabsStopSign(qlabs)
    # StopSign3.spawn(
    #     location=[-10.719, 46.669, 0.185],
    #     rotation=[0.0, 0.0, 0.0],
    #     waitForConfirmation=False,
    # )

    # StopSign4 = QLabsStopSign(qlabs)
    # StopSign4.spawn(
    #     location=[2.482, 46.673, 0.189],
    #     rotation=[0.0, 0.0, 0.0],
    #     waitForConfirmation=False,
    # )

    StopSign5 = QLabsStopSign(qlabs)
    StopSign5.spawn(
        location=[-3.195, -12.521, 0.2],
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
    TrafficLight1 = QLabsTrafficLight(qlabs)
    TrafficLight1.spawn(location=[4.78, 3.57, 0.0], rotation=[0.0, 0.0, 0.0])
    TrafficLight1.set_color(QLabsTrafficLight.COLOR_RED)

    # TrafficLight2 = QLabsTrafficLight(qlabs)
    # TrafficLight2.spawn(location=[-21.586, 14.403, 0.192], rotation=[0.0, 0.0, math.pi])
    # TrafficLight2.set_color(QLabsTrafficLight.COLOR_YELLOW)

    # TrafficLight3 = QLabsTrafficLight(qlabs)
    # TrafficLight3.spawn(location=[-21.586, 33.136, 0.182], rotation=[0.0, 0.0, math.pi])
    # TrafficLight3.set_color(QLabsTrafficLight.COLOR_RED)

    TrafficLight4 = QLabsTrafficLight(qlabs)
    TrafficLight4.spawn(location=[7.34, 13.3, 0.0], rotation=[0.0, 0.0, 1.57])
    TrafficLight4.set_color(QLabsTrafficLight.COLOR_RED)

    TrafficLight5 = QLabsTrafficLight(qlabs)
    TrafficLight5.spawn(location=[-2.1, 15.5, 0.0], rotation=[0.0, 0.0, 3.14])
    TrafficLight5.set_color(QLabsTrafficLight.COLOR_RED)
 
    TrafficLight6 = QLabsTrafficLight(qlabs)
    TrafficLight6.spawn(location=[-4.85, 6.3, 0.0], rotation=[0.0, 0.0, 4.71])
    TrafficLight6.set_color(QLabsTrafficLight.COLOR_RED)


    # Start spawn model
    # QLabsRealTime().start_real_time_model(rtModel)

    return 0


def terminate():
    QLabsRealTime().terminate_real_time_model(rtmodels.QCAR_STUDIO)


if __name__ == "__main__":
    setup()
