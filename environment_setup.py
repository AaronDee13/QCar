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
    stopsign1 = QLabsStopSign(qlabs)
    stopsign1.spawn(
        location=[24.328, 18.0, 0.0],
        rotation=[0.0, 0.0, -1.6],
        waitForConfirmation=False,
    )

    stopsign2 = QLabsStopSign(qlabs)
    stopsign2.spawn(
        location=[24.328, 2.5, 0.0],
        rotation=[0.0, 0.0, -1.6],
        waitForConfirmation=False,
    )

    stopsign3 = QLabsStopSign(qlabs)
    stopsign3.spawn(
        location=[-10.719, 46.669, 0.185],
        rotation=[0.0, 0.0, 0.0],
        waitForConfirmation=False,
    )

    stopsign4 = QLabsStopSign(qlabs)
    stopsign4.spawn(
        location=[2.482, 46.673, 0.189],
        rotation=[0.0, 0.0, 0.0],
        waitForConfirmation=False,
    )

    # spawn traffic lights
    trafficlight1 = QLabsTrafficLight(qlabs)
    trafficlight1.spawn(location=[1.108, -12.534, 0.2], rotation=[0.0, 0.0, -1.6])
    trafficlight1.set_color(QLabsTrafficLight.COLOR_GREEN)

    trafficlight2 = QLabsTrafficLight(qlabs)
    trafficlight2.spawn(location=[-21.586, 14.403, 0.192], rotation=[0.0, 0.0, math.pi])
    trafficlight2.set_color(QLabsTrafficLight.COLOR_YELLOW)

    trafficlight3 = QLabsTrafficLight(qlabs)
    trafficlight3.spawn(location=[-21.586, 33.136, 0.182], rotation=[0.0, 0.0, math.pi])
    trafficlight3.set_color(QLabsTrafficLight.COLOR_RED)

    trafficlight4 = QLabsTrafficLight(qlabs)
    trafficlight4.spawn(location=[24.271, 32.997, 0.18], rotation=[0.0, 0.0, 0.0])
    trafficlight4.set_color(QLabsTrafficLight.COLOR_RED)

    # Start spawn model
    QLabsRealTime().start_real_time_model(rtModel)

    return qcar


def terminate():
    QLabsRealTime().terminate_real_time_model(rtmodels.QCAR_STUDIO)


if __name__ == "__main__":
    setup()