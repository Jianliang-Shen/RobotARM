import time
from ArmDriver.DM_CAN import *
from ArmDriver.ArmDriver import RobotController

if __name__ == "__main__":
    controller = RobotController(port='/dev/ttyACM0', type='Master')
    controller.enable()

    while True:
        controller.gravity_compensation()
        time.sleep(0.02)
