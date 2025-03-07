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
from qvl.crosswalk import QLabsCrosswalk
import pal.resources.rtmodels as rtmodels


def terminate():
    QLabsRealTime().terminate_real_time_model(rtmodels.QCAR_STUDIO)

def main(
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
    terminate()

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

    # Spawn stop signs
    StopSign1 = QLabsStopSign(qlabs)
    StopSign1.spawn(
        location=[24.328, 18.0, 0.0],
        rotation=[0.0, 0.0, -1.6],
        waitForConfirmation=False,
    )

    StopSign2 = QLabsStopSign(qlabs)
    StopSign2.spawn(
        location=[24.328, 2.5, 0.0],
        rotation=[0.0, 0.0, -1.6],
        waitForConfirmation=False,
    )

    StopSign3 = QLabsStopSign(qlabs)
    StopSign3.spawn(
        location=[-10.719, 46.669, 0.185],
        rotation=[0.0, 0.0, 0.0],
        waitForConfirmation=False,
    )

    StopSign4 = QLabsStopSign(qlabs)
    StopSign4.spawn(
        location=[2.482, 46.673, 0.189],
        rotation=[0.0, 0.0, 0.0],
        waitForConfirmation=False,
    )
    StopSign5 = QLabsStopSign(qlabs)
    StopSign5.spawn(
        location=[-3.195, -12.521, 0.2],
        rotation=[0.0, 0.0, 3.14],
        waitForConfirmation=False,
    )
    # spawn traffic lights
    TrafficLight1 = QLabsTrafficLight(qlabs)
    TrafficLight1.spawn(location=[24.271, 32.997, 0.18], rotation=[0.0, 0.0, 0.0])
    TrafficLight1.set_state(QLabsTrafficLight.STATE_RED)

    TrafficLight2 = QLabsTrafficLight(qlabs)
    TrafficLight2.spawn(location=[-21.586, 14.403, 0.192], rotation=[0.0, 0.0, math.pi])
    TrafficLight2.set_state(QLabsTrafficLight.STATE_YELLOW)

    TrafficLight3 = QLabsTrafficLight(qlabs)
    TrafficLight3.spawn(location=[-21.586, 33.136, 0.182], rotation=[0.0, 0.0, math.pi])
    TrafficLight3.set_state(QLabsTrafficLight.STATE_RED)


    # Start spawn model
    QLabsRealTime().start_real_time_model(rtModel)

    return qcar


if __name__ == "__main__":
    main()