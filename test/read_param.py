import time
from ArmDriver.ArmDriver import RobotController

if __name__ == "__main__":
    controller = RobotController(port='/dev/ttyACM1')

    # if not controller.set_zero():
    #     print("Set Zero failed")

    # if not controller.set_mode():
    #     print("Arm mode set failed")

    # if not controller.set_mit_mode():
    #     print("Arm MIT mode set failed")

    # if not controller.set_joint_param():
    #     print("Arm joint param set failed")

    for joint in controller.joints:
        print('----------------------------------------------')
        controller.get_motor_params(joint)

    print('----------------------------------------------')
    controller.get_motor_params(controller.gripper)

    if not controller.enable():
        print("Arm enable failed")
    
    time.sleep(1)

    # while True:
    #     current_angles = controller.get_current_joint_angles()
    #     print("q:", [angle for angle in current_angles])

    #     current_gripper = controller.get_current_gripper_angles()
    #     print("gripper: ", current_gripper)

    #     time.sleep(0.1)

    if not controller.disable():
        print("Arm disable failed")