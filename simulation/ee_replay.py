import time
import math
import os
import numpy as np
from ArmDriver.RobotKinematics import RobotKinematics, MujocoRobot


def quaternion_to_euler(qx, qy, qz, qw):
    """
    将四元数转换为欧拉角 (XYZ顺序: roll, pitch, yaw)
    使用标准的四元数到欧拉角转换公式
    
    参数:
        qx, qy, qz, qw: 四元数分量 (x, y, z, w)
    
    返回:
        roll, pitch, yaw: 欧拉角 (弧度)
    """
    # 标准化四元数
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm > 0:
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm
    
    # 转换为XYZ欧拉角 (roll, pitch, yaw)
    # 参考: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角转换为四元数 (XYZ顺序: roll, pitch, yaw)
    
    参数:
        roll, pitch, yaw: 欧拉角 (弧度)
    
    返回:
        qx, qy, qz, qw: 四元数分量 (x, y, z, w)
    """
    # 计算每个轴的半角
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    # 计算四元数分量
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return qx, qy, qz, qw


def quaternion_multiply(q1, q2):
    """
    四元数乘法: q_result = q1 * q2
    
    参数:
        q1, q2: 四元数 (qx, qy, qz, qw)
    
    返回:
        qx, qy, qz, qw: 结果四元数
    """
    q1x, q1y, q1z, q1w = q1
    q2x, q2y, q2z, q2w = q2
    
    qx = q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y
    qy = q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x
    qz = q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w
    qw = q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z
    
    return qx, qy, qz, qw


def load_clamp_data(file_path):
    clamp_data = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) == 2:
                timestamp = float(parts[0])
                # 格式: 时间戳 x y z qx qy qz qw
                gripper = float(parts[1])* 0.05 / 85
                
                clamp_data.append((timestamp, gripper))
           
    return clamp_data

def load_trajectory(file_path):
    """
    读取轨迹文件
    数据格式: 时间戳 x y z qx qy qz qw
    - 时间戳: 时间戳（秒）
    - x, y, z: 末端执行器位置（米）
    - qx, qy, qz, qw: 四元数表示的姿态
    
    参数:
        file_path: 轨迹文件路径
    
    返回:
        trajectory: 列表，每个元素为 (timestamp, x, y, z, qx, qy, qz, qw)
    """
    trajectory = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 8:
                timestamp = float(parts[0])
                # 格式: 时间戳 x y z qx qy qz qw
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                qx = float(parts[4])
                qy = float(parts[5])
                qz = float(parts[6])
                qw = float(parts[7])
                trajectory.append((timestamp, x, y, z, qx, qy, qz, qw))
            elif len(parts) >= 5:
                # 如果只有时间戳和四元数（没有位置），使用默认位置
                timestamp = float(parts[0])
                qx = float(parts[1])
                qy = float(parts[2])
                qz = float(parts[3])
                qw = float(parts[4])
                # 使用初始位置（需要根据实际情况调整）
                x = y = z = 0.0
                trajectory.append((timestamp, x, y, z, qx, qy, qz, qw))
    return trajectory


if __name__ == "__main__":
    # 加载轨迹文件
    base_dir = os.path.dirname(os.path.abspath(__file__))
    trajectory_file = os.path.join(os.path.dirname(base_dir), "merged_trajectory.txt")
    clamp_data_file = os.path.join(os.path.dirname(base_dir), "clamp_data_tum.txt")
    
    print(f"正在加载轨迹文件: {trajectory_file}")
    trajectory = load_trajectory(trajectory_file)
    print(f"加载了 {len(trajectory)} 个轨迹点")

    clamp_data = load_clamp_data(clamp_data_file)
    
    if len(trajectory) == 0:
        print("错误: 轨迹文件为空")
        exit(1)
    
    # 初始化仿真和运动学
    sim = MujocoRobot()
    arm = RobotKinematics()
    
    # 初始基准位置 [x, y, z, roll, pitch, yaw]
    base_pos = [0.28242831, 0.00000000, 0.27615891, -1.57, 1.57, -1.57]
    base_x, base_y, base_z, base_roll, base_pitch, base_yaw = base_pos
    
    # 将初始基准欧拉角转换为四元数
    base_qx, base_qy, base_qz, base_qw = euler_to_quaternion(base_roll, base_pitch, base_yaw)
    
    print(f"初始基准位置: x={base_x:.6f}, y={base_y:.6f}, z={base_z:.6f}")
    print(f"初始基准姿态 (欧拉角): roll={base_roll:.6f}, pitch={base_pitch:.6f}, yaw={base_yaw:.6f}")
    
    # 初始化关节角度
    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    q_old = q.copy()
    gripper = 0.0
    
    # 使用初始基准位置计算初始逆运动学解
    q, ans_index = arm.InverseKinematics(arm.p2r_matrix(*base_pos), q)
    if q:
        q_old = q.copy()
        pos_old = base_pos.copy()
        sim.update_param(q, gripper)
        print(f"初始位置设置完成: pos={base_pos}, q={q}")
    else:
        print("警告: 初始位置无法求解逆运动学，使用默认位置")
    
    # 获取第一个轨迹点的时间戳作为起始时间
    if len(trajectory) > 0:
        prev_timestamp, _, _, _, _, _, _, _ = trajectory[0]
    else:
        prev_timestamp = 0.0
    trajectory_index = 0  # 从第一个轨迹点开始
    clamp_data_index = 0  # 从第一个轨迹点开始
    
    print("开始回放轨迹...")
    start_time = time.time()
    
    while sim.viewer.is_running() and trajectory_index < len(trajectory):
        # 获取当前轨迹点（相对值）
        timestamp, rel_x, rel_y, rel_z, rel_qx, rel_qy, rel_qz, rel_qw = trajectory[trajectory_index]
        timestamp_2, gripper =  clamp_data[trajectory_index * 2]
        
        # 计算时间间隔
        time_delta = timestamp - prev_timestamp
        
        # 位置：相对值 + 基准值
        x = base_x + rel_x
        y = base_y + rel_y
        z = base_z + rel_z
        
        # 姿态：将相对四元数与基准四元数相乘（四元数乘法表示旋转的组合）
        base_quat = (base_qx, base_qy, base_qz, base_qw)
        rel_quat = (rel_qx, rel_qy, rel_qz, rel_qw)
        combined_qx, combined_qy, combined_qz, combined_qw = quaternion_multiply(base_quat, rel_quat)
        
        # 将组合后的四元数转换为欧拉角
        roll, pitch, yaw = quaternion_to_euler(combined_qx, combined_qy, combined_qz, combined_qw)
        
        # 组合位置和姿态
        pos = [x, y, z, roll, pitch, yaw]
        
        # 计算逆运动学
        q, ans_index = arm.InverseKinematics(arm.p2r_matrix(*pos), q_old)
        
        # 如果有解，检查边界
        if q:
            if arm.check_range(arm.joint_range, q):
                # 更新位置和关节角度
                pos_old = pos.copy()
                q_old = q.copy()

                q[5] -= 1.57
                
                # 更新仿真
                sim.update_param(q, gripper)
                
                # 打印信息（可选，可以注释掉以减少输出）
                if trajectory_index % 100 == 0:  # 每10个点打印一次
                    print(f"轨迹点 {trajectory_index}/{len(trajectory)}, "
                          f"时间戳: {timestamp:.6f}, "
                          f"相对位置: [{rel_x:.6f}, {rel_y:.6f}, {rel_z:.6f}], "
                          f"绝对位置: [{x:.6f}, {y:.6f}, {z:.6f}], "
                          f"绝对欧拉角: [{roll:.6f}, {pitch:.6f}, {yaw:.6f}]")
            else:
                # 如果越界，保持原位置
                pos = pos_old.copy()
                q = q_old.copy()
                print(f"警告: 轨迹点 {trajectory_index} 超出关节范围，跳过")
        else:
            # 如果无解，保持原位置
            pos = pos_old.copy()
            q = q_old.copy()
            print(f"警告: 轨迹点 {trajectory_index} 无法求解逆运动学，跳过")
        
        # 按照时间戳间隔等待
        # 注意: 如果时间间隔太小，使用最小间隔
        sleep_time = max(time_delta, 0.001)  # 最小1ms
        time.sleep(sleep_time)
        
        prev_timestamp = timestamp
        trajectory_index += 1
    
    print(f"轨迹回放完成，共处理 {trajectory_index} 个轨迹点")
