import time
from pynput import keyboard
from ArmDriver.RobotKinematics import RobotKinematics, MujocoRobot

key_map = {
    "1": (0, +1), "q": (0, -1),  # joint1
    "s": (1, +1), "z": (1, -1),  # joint2
    "3": (2, +1), "e": (2, -1),  # joint3
    "4": (3, +1), "r": (3, -1),  # joint4
    "5": (4, +1), "t": (4, -1),  # joint5
    "6": (5, +1), "y": (5, -1),  # joint6
    "7": (6, +1), "8": (6, -1),  # gripper
}

pressed_keys = set()

def on_press(key):
    try:
        if key.char in key_map:
            pressed_keys.add(key.char)
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char in pressed_keys:
            pressed_keys.remove(key.char)
    except AttributeError:
        pass

if __name__ == "__main__":
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    sim = MujocoRobot()
    arm = RobotKinematics()

    q = [0, 0, 0, 0, 0, 0]
    gripper = 0

    while sim.viewer.is_running():
        keys = pressed_keys.copy()
        if keys:
            for k in keys:
                if k in key_map:
                    jid, direction = key_map[k]
                    if jid < 6:
                        q[jid] += direction * sim.joint_step_size
                        q[jid] = arm.set_boundary(arm.joint_range, q[jid], jid)
                    elif jid == 6:
                        gripper += direction * sim.gripper_step_size
                        gripper = arm.set_boundary(arm.joint_range, gripper, jid)
        

        result_matrix = arm.ForwardKinematics(q)
        pos = arm.extract_pose(result_matrix)
        print(f"q: {' '.join([f'{val:.3f}' for val in q])}, pos: {' '.join([f'{val:.8f}' for val in pos])}, gripper: {gripper: .3f}")

        sim.update_param(q, gripper)
        time.sleep(0.01)
