import time
from pynput import keyboard
from ArmDriver.RobotKinematics import RobotKinematics, MujocoRobot

key_map = {
    "1": (0, +1), "q": (0, -1),  # x
    "s": (1, +1), "z": (1, -1),  # y
    "3": (2, +1), "e": (2, -1),  # z
    "4": (3, +1), "r": (3, -1),  # roll
    "5": (4, +1), "t": (4, -1),  # pitch
    "6": (5, +1), "y": (5, -1),  # yaw
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

    pos = [0.18642831, 0.00000000, 0.18815891, -1.57079633, 0, -1.57079633]
    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    q_old = q.copy()
    pos_old = pos.copy()
    gripper = 0.0

    while sim.viewer.is_running():
        keys = pressed_keys.copy()
        if keys:
            for k in keys:
                if k in key_map:
                    jid, direction = key_map[k]
                    if jid < 3:
                        pos[jid] += direction * sim.ee_step_size
                    elif jid >= 3 and jid < 6:
                        pos[jid] += direction * sim.ee_roll_size
                    elif jid == 6:
                        gripper += direction * sim.gripper_step_size
                        gripper = arm.set_boundary(arm.joint_range, gripper, jid)
        

        q, ans_index = arm.InverseKinematics(arm.p2r_matrix(*pos), q)

        # 如果有解，则检查边界：如果越界，则不渲染（运动），保持原有位置不动；
        # 如果未越界，则渲染（运动），更新原有位置（至最新位置）
        # 如果无解，不渲染（运动），保持原有位置不动
        if q:
            if not arm.check_range(arm.joint_range, q):
                pos = pos_old.copy()
                q = q_old.copy()
            else:
                pos_old = pos.copy()
                q_old = q.copy()

            print(f"ans_index = {ans_index}, q: {' '.join([f'{val:.6f}' for val in q])}, pos: {' '.join([f'{val:.8f}' for val in pos])}, gripper: {gripper: .3f}")

            sim.update_param(q, gripper)
        else:
            pos = pos_old.copy()
            q = q_old.copy()

        time.sleep(0.01)
