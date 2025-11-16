import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
from pynput import keyboard
import math

from joyconrobotics import JoyconRobotics
import time

# 继承JoyconRobotics类来修改控制逻辑
class FixedAxesJoyconRobotics(JoyconRobotics):
    def __init__(self, 
                 device="right", 
                 joycon_stick_v_0=2300, 
                 joycon_stick_h_0=2000, 
                 **kwargs):
        # 调用父类构造函数
        super().__init__(device, **kwargs)
        
        # 存储摇杆中心值
        self.joycon_stick_v_0 = joycon_stick_v_0
        self.joycon_stick_h_0 = joycon_stick_h_0
    
    def common_update(self):
        # 修改后的更新逻辑：摇杆只控制固定轴向
        
        # 垂直摇杆：只控制X轴（前后）
        joycon_stick_v = self.joycon.get_stick_right_vertical() if self.joycon.is_right() else self.joycon.get_stick_left_vertical()
        joycon_stick_v_threshold = 300
        joycon_stick_v_range = 1000
        # print(f"joycon_stick_v: {joycon_stick_v}")
        if joycon_stick_v > joycon_stick_v_threshold + self.joycon_stick_v_0:
            self.position[0] += 0.001 * (joycon_stick_v - self.joycon_stick_v_0) / joycon_stick_v_range *self.dof_speed[0] 
        elif joycon_stick_v < self.joycon_stick_v_0 - joycon_stick_v_threshold:
            self.position[0] += 0.001 * (joycon_stick_v - self.joycon_stick_v_0) / joycon_stick_v_range *self.dof_speed[0] 
        
        # 水平摇杆：只控制Y轴（左右）  
        joycon_stick_h = self.joycon.get_stick_right_horizontal() if self.joycon.is_right() else self.joycon.get_stick_left_horizontal()
        joycon_stick_h_threshold = 300
        joycon_stick_h_range = 1000
        # print(f"stick_h: {joycon_stick_h}, stick_v: {joycon_stick_v}")
        if joycon_stick_h > joycon_stick_h_threshold + self.joycon_stick_h_0:
            self.position[1] += 0.001 * (joycon_stick_h - self.joycon_stick_h_0) / joycon_stick_h_range * self.dof_speed[1] * self.direction_reverse[1]
        elif joycon_stick_h < self.joycon_stick_h_0 - joycon_stick_h_threshold:
            self.position[1] += 0.001 * (joycon_stick_h - self.joycon_stick_h_0) / joycon_stick_h_range * self.dof_speed[1] * self.direction_reverse[1]
        
        # Z轴只通过按钮控制
        joycon_button_up = self.joycon.get_button_r() if self.joycon.is_right() else self.joycon.get_button_l()
        if joycon_button_up == 1:
            self.position[2] += 0.001 * self.dof_speed[2] 
        
        joycon_button_down = self.joycon.get_button_r_stick() if self.joycon.is_right() else self.joycon.get_button_l_stick()
        if joycon_button_down == 1:
            self.position[2] -= 0.001 * self.dof_speed[2] 

        # 其他按钮控制（复制原来的逻辑）
        joycon_button_xup = self.joycon.get_button_x() if self.joycon.is_right() else self.joycon.get_button_up()
        joycon_button_xback = self.joycon.get_button_b() if self.joycon.is_right() else self.joycon.get_button_down()
        if joycon_button_xup == 1:
            self.position[0] += 0.001 * self.dof_speed[0]
        elif joycon_button_xback == 1:
            self.position[0] -= 0.001 * self.dof_speed[0]
        
        # Home按钮重置逻辑（简化版）
        joycon_button_home = self.joycon.get_button_home() if self.joycon.is_right() else self.joycon.get_button_capture()
        if joycon_button_home == 1:
            self.position = self.offset_position_m.copy()
        
        # 夹爪控制逻辑（复制原来的）
        for event_type, status in self.button.events():
            if (self.joycon.is_right() and event_type == 'plus' and status == 1) or (self.joycon.is_left() and event_type == 'minus' and status == 1):
                self.reset_button = 1
                self.reset_joycon()
            elif self.joycon.is_right() and event_type == 'a':
                self.next_episode_button = status
            elif self.joycon.is_right() and event_type == 'y':
                self.restart_episode_button = status
            elif ((self.joycon.is_right() and event_type == 'zr') or (self.joycon.is_left() and event_type == 'zl')) and not self.change_down_to_gripper:
                self.gripper_toggle_button = status
            elif ((self.joycon.is_right() and event_type == 'stick_r_btn') or (self.joycon.is_left() and event_type == 'stick_l_btn')) and self.change_down_to_gripper:
                self.gripper_toggle_button = status
            else: 
                self.reset_button = 0
            
        if self.gripper_toggle_button == 1 :
            if self.gripper_state == self.gripper_open:
                self.gripper_state = self.gripper_close
            else:
                self.gripper_state = self.gripper_open
            self.gripper_toggle_button = 0

        # 按钮控制状态
        if self.joycon.is_right():
            if self.next_episode_button == 1:
                self.button_control = 1
            elif self.restart_episode_button == 1:
                self.button_control = -1
            elif self.reset_button == 1:
                self.button_control = 8
            else:
                self.button_control = 0
        
        return self.position, self.gripper_state, self.button_control

# 读取机器人模型
model = mujoco.MjModel.from_xml_path("urdf/arm.xml")
data = mujoco.MjData(model)
q_des = np.copy(data.qpos)

ee_step_size = 0.008
ee_roll_size = 0.015
gripper_step_size = 0.002


# DHToMatrix 函数
def DHToMatrix(alpha, theta, a, d):
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)

    # DH 变换矩阵
    matrix = np.array([
        [   ct,    -st,    0,      a       ],
        [   st*ca, ct*ca,  -sa,    -d*sa   ],
        [   st*sa, ct*sa,  ca,     d*ca    ],
        [   0,     0,      0,      1       ]
    ])
    return matrix

# DH 参数类，存储 alpha, theta, a, d 的值
class DHParam:
    def __init__(self, alpha, theta, a, d):
        self.alpha = alpha
        self.theta = theta
        self.a = a
        self.d = d

def ForwardKinematics(DH_params, theta_input=None):
    # 初始变换矩阵（单位矩阵）
    transform = np.eye(4)

    # 遍历所有的 DH 参数，计算每一关节的变换
    for i in range(len(DH_params)):
        theta = theta_input[i] + DH_params[i].theta if theta_input is not None else DH_params[i].theta
        T = DHToMatrix(DH_params[i].alpha, theta, DH_params[i].a, DH_params[i].d)
        transform = np.dot(transform, T)

    return transform

# 从变换矩阵提取位置和姿态，返回6维数组 [x, y, z, roll, pitch, yaw]
def extract_pose(transform):
    pose = np.zeros(6)

    # 提取位置信息
    pose[0] = transform[0, 3]  # X
    pose[1] = transform[1, 3]  # Y
    pose[2] = transform[2, 3]  # Z

    # 提取姿态信息 (ZYX欧拉角)
    sy = math.sqrt(transform[0, 0] * transform[0, 0] + transform[1, 0] * transform[1, 0])

    singular = sy < 1e-6  # 奇异情况判断

    if not singular:
        pose[3] = math.atan2(transform[2, 1], transform[2, 2])  # Roll
        pose[4] = math.atan2(-transform[2, 0], sy)              # Pitch
        pose[5] = math.atan2(transform[1, 0], transform[0, 0])  # Yaw
    else:
        pose[3] = 0
        pose[4] = math.atan2(-transform[2, 0], sy)
        pose[5] = math.atan2(-transform[1, 2], transform[1, 1])

    return pose

# 角度转换为弧度
def degrees_to_radians(degrees):
    return degrees * math.pi / 180

# 创建绕X轴的旋转矩阵
def create_rotation_matrix_x(alpha):
    return np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha), math.cos(alpha)]
    ])

# 创建绕Y轴的旋转矩阵
def create_rotation_matrix_y(beta):
    return np.array([
        [math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
    ])

# 创建绕Z轴的旋转矩阵
def create_rotation_matrix_z(gamma):
    return np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma), math.cos(gamma), 0],
        [0, 0, 1]
    ])

# 创建末端姿态矩阵
def create_end_effector_pose(R, xd, yd, zd):
    return np.array([
        [R[0, 0], R[0, 1], R[0, 2], xd],
        [R[1, 0], R[1, 1], R[1, 2], yd],
        [R[2, 0], R[2, 1], R[2, 2], zd],
        [0, 0, 0, 1]
    ])

# 把末端位置和姿态转换为4x4变换矩阵
def p2r_matrix(x1, x2, x3, x4, x5, x6):
    # alpha = degrees_to_radians(x4)  # 绕X轴旋转
    # beta = degrees_to_radians(x5)   # 绕Y轴旋转
    # gamma = degrees_to_radians(x6)  # 绕Z轴旋转

    alpha = x4
    beta = x5
    gamma = x6

    # 假设的末端位置坐标
    xd = x1
    yd = x2
    zd = x3

    # 计算各个轴的旋转矩阵
    Rx = create_rotation_matrix_x(alpha)
    Ry = create_rotation_matrix_y(beta)
    Rz = create_rotation_matrix_z(gamma)

    # 计算总旋转矩阵（注意顺序：先Z，再Y，最后X）
    R = np.dot(np.dot(Rz, Ry), Rx)
    A = create_end_effector_pose(R, xd, yd, zd)

    return A

curDH = [
    DHParam(    0,          0,                  0,          0.1205  ),
    DHParam(    np.pi/2,    np.pi,              0,          0       ),
    DHParam(    0,          -168.24/180*np.pi,  0.37468,    0       ),
    DHParam(    0,          -8.15/180*np.pi,    0.28486,    0       ),
    DHParam(    -np.pi/2,   -np.pi/2,           0.08173,    0       ),
    DHParam(    -np.pi/2,   0,                  0,          0.206   ),
]


joint_range = [
    [   -np.pi / 2,     np.pi / 2   ],
    [   -np.pi,     0       ],
    [   0,          np.pi   ],
    [   -1.8,       1.4     ],
    [   -2,         2       ],
    [   -np.pi,     np.pi   ],
    [   -0.05,      0       ],
    [   0,          0.05    ]
]
def set_boundary(joint_range, input_q, idx):
    input_q = min(max(input_q, joint_range[idx][0]), joint_range[idx][1])
    return input_q

def check_range(joint_range, q):
    ret = True
    for i in range(len(q)):
        if q[i] - joint_range[i][1] > 0.001 or joint_range[i][0] - q[i] > 0.001:
            print("joint %d out of range" % (i + 1))
            ret = False
            break
    return ret

def NormalizeJointAngles(jointAngles):
    res = [0] * len(jointAngles)
    for i in range(len(jointAngles)):
        angle = jointAngles[i]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle <= -math.pi:
            angle += 2 * math.pi
        res[i] = angle
    return res

def InverseKinematics(matrix, x):
    # x 表示选择第几组解

    nx, ox, ax, px = matrix[0]
    ny, oy, ay, py = matrix[1]
    nz, oz, az, pz = matrix[2]

    d1 = curDH[0].d
    d2 = curDH[1].d
    d3 = curDH[2].d
    d4 = curDH[3].d
    d5 = curDH[4].d
    d6 = curDH[5].d

    a2 = curDH[2].a
    a3 = curDH[3].a
    a4 = curDH[4].a

    # ---- q1 ----
    m1 = d6 * ax - px
    n1 = d6 * ay - py

    q1_1 = math.atan2(n1, m1)
    q1_2 = q1_1 - math.pi if q1_1 >= 0 else q1_1 + math.pi

    # ---- q5 (对应 q1_1) ----
    sinq5_1 = ay * math.cos(q1_1) - ax * math.sin(q1_1)
    if abs(sinq5_1) <= 1:
        q5_1 = math.asin(sinq5_1)
        q5_2 = math.pi - q5_1 if q5_1 >= 0 else -math.pi - q5_1
    else:
        q5_1 = float('nan')
        q5_2 = float('nan')

    # ---- q5 (q1_2对应) ----
    sinq5_2 = ay * math.cos(q1_2) - ax * math.sin(q1_2)
    if abs(sinq5_2) <= 1:
        q5_3 = math.asin(sinq5_2)
        q5_4 = math.pi - q5_3 if q5_3 >= 0 else -math.pi - q5_3
    else:
        q5_3 = float('nan')
        q5_4 = float('nan')

    # ---- q6 (q1_1对应) ----
    u6_1 = ox * math.sin(q1_1) - oy * math.cos(q1_1)
    v6_1 = ny * math.cos(q1_1) - nx * math.sin(q1_1)

    if abs(sinq5_1) <= 1:
        q6_1 = math.atan2((-math.cos(q5_1)) / math.sqrt(u6_1**2 + v6_1**2), 0) - math.atan2(v6_1, u6_1)
        q6_2 = math.atan2((-math.cos(q5_2)) / math.sqrt(u6_1**2 + v6_1**2), 0) - math.atan2(v6_1, u6_1)
    else:
        q6_1 = float('nan')
        q6_2 = float('nan')

    # ---- q6 (q1_2对应) ----
    u6_2 = ox * math.sin(q1_2) - oy * math.cos(q1_2)
    v6_2 = ny * math.cos(q1_2) - nx * math.sin(q1_2)

    if abs(sinq5_2) <= 1:
        q6_3 = math.atan2((-math.cos(q5_3)) / math.sqrt(u6_2**2 + v6_2**2), 0) - math.atan2(v6_2, u6_2)
        q6_4 = math.atan2((-math.cos(q5_4)) / math.sqrt(u6_2**2 + v6_2**2), 0) - math.atan2(v6_2, u6_2)
    else:
        q6_3 = float('nan')
        q6_4 = float('nan')

    # ---- q3 (这里只写 q1_1 对应前两个) ----

    t14_1 = (px - ax * d6 - a4 * (ax * math.cos(q5_1) + nx * math.sin(q5_1) * math.cos(q6_1) - ox * math.sin(q5_1) * math.sin(q6_1))) * math.cos(q1_1) \
          + (py - ay * d6 - a4 * (ay * math.cos(q5_1) + ny * math.sin(q5_1) * math.cos(q6_1) - oy * math.sin(q5_1) * math.sin(q6_1))) * math.sin(q1_1)

    t34_1 = -a4 * az * math.cos(q5_1) \
          - a4 * nz * math.sin(q5_1) * math.cos(q6_1) \
          + a4 * oz * math.sin(q5_1) * math.sin(q6_1) \
          - az * d6 - d1 + pz

    t14_2 = (px - ax * d6 - a4 * (ax * math.cos(q5_2) + nx * math.sin(q5_2) * math.cos(q6_2) - ox * math.sin(q5_2) * math.sin(q6_2))) * math.cos(q1_1) \
          + (py - ay * d6 - a4 * (ay * math.cos(q5_2) + ny * math.sin(q5_2) * math.cos(q6_2) - oy * math.sin(q5_2) * math.sin(q6_2))) * math.sin(q1_1)

    t34_2 = -a4 * az * math.cos(q5_2) \
          - a4 * nz * math.sin(q5_2) * math.cos(q6_2) \
          + a4 * oz * math.sin(q5_2) * math.sin(q6_2) \
          - az * d6 - d1 + pz

    cosq3_1 = (t14_1**2 + t34_1**2 - a2**2 - a3**2) / (2 * a2 * a3)
    cosq3_2 = (t14_2**2 + t34_2**2 - a2**2 - a3**2) / (2 * a2 * a3)

    if abs(sinq5_1) <= 1:
        if abs(cosq3_1) <= 1:
            q3_1 = math.acos(cosq3_1) - curDH[2].theta
            q3_2 = -math.acos(cosq3_1) - curDH[2].theta
        else:
            q3_1 = float('nan')
            q3_2 = float('nan')

        if abs(cosq3_2) <= 1:
            q3_3 = math.acos(cosq3_2) - curDH[2].theta
            q3_4 = -math.acos(cosq3_2) - curDH[2].theta
        else:
            q3_3 = float('nan')
            q3_4 = float('nan')
    else:
        q3_1 = q3_2 = q3_3 = q3_4 = float('nan')

    t14_3 = (px - ax * d6 - a4 * (ax * math.cos(q5_3) + nx * math.sin(q5_3) * math.cos(q6_3) - ox * math.sin(q5_3) * math.sin(q6_3))) * math.cos(q1_2) \
          + (py - ay * d6 - a4 * (ay * math.cos(q5_3) + ny * math.sin(q5_3) * math.cos(q6_3) - oy * math.sin(q5_3) * math.sin(q6_3))) * math.sin(q1_2)

    t34_3 = -a4 * az * math.cos(q5_3) \
          - a4 * nz * math.sin(q5_3) * math.cos(q6_3) \
          + a4 * oz * math.sin(q5_3) * math.sin(q6_3) \
          - az * d6 - d1 + pz

    t14_4 = (px - ax * d6 - a4 * (ax * math.cos(q5_4) + nx * math.sin(q5_4) * math.cos(q6_4) - ox * math.sin(q5_4) * math.sin(q6_4))) * math.cos(q1_2) \
          + (py - ay * d6 - a4 * (ay * math.cos(q5_4) + ny * math.sin(q5_4) * math.cos(q6_4) - oy * math.sin(q5_4) * math.sin(q6_4))) * math.sin(q1_2)

    t34_4 = -a4 * az * math.cos(q5_4) \
          - a4 * nz * math.sin(q5_4) * math.cos(q6_4) \
          + a4 * oz * math.sin(q5_4) * math.sin(q6_4) \
          - az * d6 - d1 + pz

    cosq3_3 = (t14_3**2 + t34_3**2 - a2**2 - a3**2) / (2 * a2 * a3)
    cosq3_4 = (t14_4**2 + t34_4**2 - a2**2 - a3**2) / (2 * a2 * a3)

    if abs(sinq5_2) <= 1:
        if abs(cosq3_3) <= 1:
            q3_5 = math.acos(cosq3_3) - curDH[2].theta
            q3_6 = -math.acos(cosq3_3) - curDH[2].theta
        else:
            q3_5 = float('nan'); q3_6 = float('nan')

        if abs(cosq3_4) <= 1:
            q3_7 = math.acos(cosq3_4) - curDH[2].theta
            q3_8 = -math.acos(cosq3_4) - curDH[2].theta
        else:
            q3_7 = float('nan'); q3_8 = float('nan')
    else:
        q3_5 = q3_6 = q3_7 = q3_8 = float('nan')
    
    g2_1 = a3 * math.sin(q3_1 + curDH[2].theta)
    h2_1 = a2 + a3 * math.cos(q3_1 + curDH[2].theta)

    g2_2 = a3 * math.sin(q3_2 + curDH[2].theta)
    h2_2 = a2 + a3 * math.cos(q3_2 + curDH[2].theta)

    g2_3 = a3 * math.sin(q3_3 + curDH[2].theta)
    h2_3 = a2 + a3 * math.cos(q3_3 + curDH[2].theta)

    g2_4 = a3 * math.sin(q3_4 + curDH[2].theta)
    h2_4 = a2 + a3 * math.cos(q3_4 + curDH[2].theta)

    if abs(sinq5_1) <= 1:
        q2_1 = math.atan2(h2_1 * t34_1 - g2_1 * t14_1, h2_1 * t14_1 + g2_1 * t34_1) - curDH[1].theta
        q2_2 = math.atan2(h2_2 * t34_1 - g2_2 * t14_1, h2_2 * t14_1 + g2_2 * t34_1) - curDH[1].theta
        q2_3 = math.atan2(h2_3 * t34_2 - g2_3 * t14_2, h2_3 * t14_2 + g2_3 * t34_2) - curDH[1].theta
        q2_4 = math.atan2(h2_4 * t34_2 - g2_4 * t14_2, h2_4 * t14_2 + g2_4 * t34_2) - curDH[1].theta
    else:
        q2_1 = float('nan')
        q2_2 = float('nan')
        q2_3 = float('nan')
        q2_4 = float('nan')

    g2_5 = a3 * math.sin(q3_5 + curDH[2].theta)
    h2_5 = a2 + a3 * math.cos(q3_5 + curDH[2].theta)

    g2_6 = a3 * math.sin(q3_6 + curDH[2].theta)
    h2_6 = a2 + a3 * math.cos(q3_6 + curDH[2].theta)

    g2_7 = a3 * math.sin(q3_7 + curDH[2].theta)
    h2_7 = a2 + a3 * math.cos(q3_7 + curDH[2].theta)

    g2_8 = a3 * math.sin(q3_8 + curDH[2].theta)
    h2_8 = a2 + a3 * math.cos(q3_8 + curDH[2].theta)

    if abs(sinq5_2) <= 1:
        q2_5 = math.atan2(h2_5 * t34_3 - g2_5 * t14_3, h2_5 * t14_3 + g2_5 * t34_3) - curDH[1].theta
        q2_6 = math.atan2(h2_6 * t34_3 - g2_6 * t14_3, h2_6 * t14_3 + g2_6 * t34_3) - curDH[1].theta
        q2_7 = math.atan2(h2_7 * t34_4 - g2_7 * t14_4, h2_7 * t14_4 + g2_7 * t34_4) - curDH[1].theta
        q2_8 = math.atan2(h2_8 * t34_4 - g2_8 * t14_4, h2_8 * t14_4 + g2_8 * t34_4) - curDH[1].theta
    else:
        q2_5 = float('nan')
        q2_6 = float('nan')
        q2_7 = float('nan')
        q2_8 = float('nan')

    sin234_1 = -math.cos(q6_1) * (ox * math.cos(q1_1) + oy * math.sin(q1_1)) \
            - math.sin(q6_1) * (nx * math.cos(q1_1) + ny * math.sin(q1_1))

    sin234_2 = -math.cos(q6_2) * (ox * math.cos(q1_1) + oy * math.sin(q1_1)) \
            - math.sin(q6_2) * (nx * math.cos(q1_1) + ny * math.sin(q1_1))

    cos234_1 = oz * math.cos(q6_1) + nz * math.sin(q6_1)
    cos234_2 = oz * math.cos(q6_2) + nz * math.sin(q6_2)

    if abs(sinq5_1) <= 1:
        q4_1 = math.atan2(sin234_1, cos234_1) - q2_1 - q3_1 - curDH[1].theta
        q4_2 = math.atan2(sin234_1, cos234_1) - q2_2 - q3_2 - curDH[1].theta
        q4_3 = math.atan2(sin234_2, cos234_2) - q2_3 - q3_3 - curDH[1].theta
        q4_4 = math.atan2(sin234_2, cos234_2) - q2_4 - q3_4 - curDH[1].theta
    else:
        q4_1 = q4_2 = q4_3 = q4_4 = float('nan')

    sin234_3 = -math.cos(q6_3) * (ox * math.cos(q1_2) + oy * math.sin(q1_2)) \
            - math.sin(q6_3) * (nx * math.cos(q1_2) + ny * math.sin(q1_2))

    sin234_4 = -math.cos(q6_4) * (ox * math.cos(q1_2) + oy * math.sin(q1_2)) \
            - math.sin(q6_4) * (nx * math.cos(q1_2) + ny * math.sin(q1_2))

    cos234_3 = oz * math.cos(q6_3) + nz * math.sin(q6_3)
    cos234_4 = oz * math.cos(q6_4) + nz * math.sin(q6_4)

    if abs(sinq5_2) <= 1:
        q4_5 = math.atan2(sin234_3, cos234_3) - q2_5 - q3_5 - curDH[1].theta
        q4_6 = math.atan2(sin234_3, cos234_3) - q2_6 - q3_6 - curDH[1].theta
        q4_7 = math.atan2(sin234_4, cos234_4) - q2_7 - q3_7 - curDH[1].theta
        q4_8 = math.atan2(sin234_4, cos234_4) - q2_8 - q3_8 - curDH[1].theta
    else:
        q4_5 = q4_6 = q4_7 = q4_8 = float('nan')
    
    # ---------- 汇总结果 ----------
    result = []

    if not (math.isnan(q1_1) or math.isnan(q2_1) or math.isnan(q3_1) or math.isnan(q4_1) or math.isnan(q5_1) or math.isnan(q6_1)):
        result.append(NormalizeJointAngles([q1_1, q2_1, q3_1, q4_1, q5_1, q6_1]))

    if not (math.isnan(q1_1) or math.isnan(q2_2) or math.isnan(q3_2) or math.isnan(q4_2) or math.isnan(q5_1) or math.isnan(q6_1)):
        result.append(NormalizeJointAngles([q1_1, q2_2, q3_2, q4_2, q5_1, q6_1]))

    if not (math.isnan(q1_1) or math.isnan(q2_3) or math.isnan(q3_3) or math.isnan(q4_3) or math.isnan(q5_2) or math.isnan(q6_2)):
        result.append(NormalizeJointAngles([q1_1, q2_3, q3_3, q4_3, q5_2, q6_2]))

    if not (math.isnan(q1_1) or math.isnan(q2_4) or math.isnan(q3_4) or math.isnan(q4_4) or math.isnan(q5_2) or math.isnan(q6_2)):
        result.append(NormalizeJointAngles([q1_1, q2_4, q3_4, q4_4, q5_2, q6_2]))

    if not (math.isnan(q1_2) or math.isnan(q2_5) or math.isnan(q3_5) or math.isnan(q4_5) or math.isnan(q5_3) or math.isnan(q6_3)):
        result.append(NormalizeJointAngles([q1_2, q2_5, q3_5, q4_5, q5_3, q6_3]))

    if not (math.isnan(q1_2) or math.isnan(q2_6) or math.isnan(q3_6) or math.isnan(q4_6) or math.isnan(q5_3) or math.isnan(q6_3)):
        result.append(NormalizeJointAngles([q1_2, q2_6, q3_6, q4_6, q5_3, q6_3]))

    if not (math.isnan(q1_2) or math.isnan(q2_7) or math.isnan(q3_7) or math.isnan(q4_7) or math.isnan(q5_4) or math.isnan(q6_4)):
        result.append(NormalizeJointAngles([q1_2, q2_7, q3_7, q4_7, q5_4, q6_4]))

    if not (math.isnan(q1_2) or math.isnan(q2_8) or math.isnan(q3_8) or math.isnan(q4_8) or math.isnan(q5_4) or math.isnan(q6_4)):
        result.append(NormalizeJointAngles([q1_2, q2_8, q3_8, q4_8, q5_4, q6_4]))

    select = None
    min_d = 1000
    min_d_idx = 0
    
    def get_d(ans, q):
        ret = 0
        for i in range(6):
            ret += abs(ans[i] - q[i])

        return ret

    if len(result) > 0:
        for i in range (len(result)):
            ans = result[i]
            d = get_d(ans, q)
            if d < min_d:
                min_d = d
                min_d_idx = i
                select = ans
    
    return select, min_d_idx


pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q_old = q.copy()
pos_old = pos.copy()

joyconrobotics_right = FixedAxesJoyconRobotics(
    device="right",
    joycon_stick_v_0=1900,  # 垂直摇杆中心值
    joycon_stick_h_0=2100,  # 水平摇杆中心值
    dof_speed=[2, 2, 2, 1, 1, 1]
)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        pose_right, gripper_right, control_button_right = joyconrobotics_right.get_control()  # 改变量名
        x_right, y_right, z_right, roll_right, pitch_right, yaw_right = pose_right
        # print(f'pos_right={x_right:.3f}, {y_right:.3f}, {z_right:.3f}, Rot_right={roll_right:.3f}, {pitch_right:.3f}, {yaw_right:.3f}, gripper_right={gripper_right}, control_button_right={control_button_right}')
        
        pos[0] = 0.56611615 + pose_right[0]
        pos[1] = 0.00000000 + pose_right[1]
        pos[2] = 0.57135489 + pose_right[2]
        pos[3] = -1.50778994 - pitch_right
        pos[4] = 0.00000000 - roll_right
        pos[5] = -1.57079633 + yaw_right

        q, ans_index = InverseKinematics(p2r_matrix(*pos), q)

        if q:
            if not check_range(joint_range, q):
                pos = pos_old.copy()
                q = q_old.copy()

            else:
                pos_old = pos.copy()
                q_old = q.copy()

            print(f"ans_index = {ans_index}, q: {' '.join([f'{val:.3f}' for val in q])}, pos: {' '.join([f'{val:.3f}' for val in pos])}, gripper: {-data.qpos[13]: .3f}")

            data.qpos[:len(q_des)] = q_des
            data.qvel[:] = 0
            data.qpos[7:13] = q
            mujoco.mj_step(model, data)
            viewer.sync()

        else:
            pos = pos_old.copy()
            q = q_old.copy()

        time.sleep(0.02)

joyconrobotics_right.disconnect()
