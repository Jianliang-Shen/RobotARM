
# 项目结构

- Host:
  - RobotArm_V2_Controller_V5.0 : 上位机源码程序
  - PythonPosVel : 达妙电机 Python 驱动程序
- Device
  - CtrlBoard : Firmware
  - scripts : 测试
- DmArm
  - ArmDriver.py : 通信协议
  - follower.py : 从臂（达妙臂）
  - leader.py : 主臂（uarm 舵机臂）
- tests
  - test_<test_name>.py : 针对 ArmDriver.py 的测试，函数接口调用示例
  - lerobot_telecontrol.py : 测试 lerobot 遥操作（不显示相机，可达到 50 帧）

# 运行环境

支持多平台, MacOS, Windows, Linux.

运行环境依赖:

1. 安装 lerobot
  
    ```bash
    conda create -n lerobot python==3.10
    conda activate lerobot

    git clone -b dev git@github.com:Jianliang-Shen/lerobot.git
    cd lerobot
    pip install -e .
    ```

2. 安装 DmArm

    ```bash
    git clone git@github.com:MINT-SJTU/Robotic-Arm-Control-Program.git
    cd Robotic-Arm-Control-Program
    pip install -e .

    # additional packages
    pip install pyserial IPython matplotlib
    ```

3. 从臂烧写固件 : 使用 STM32CubeMXIDE (略)
4. 从臂设置零位 : 借助 Host 上位机，确保初始角度为：**`[0, -3.1, 1.6, 0, 0, 0, 0]`**
5. 命令行交互

    ```bash
    sudo chmod 666 /dev/ttyACM0
    python tests/PySerial.py --port /dev/ttyACM0
    ```

    > **注意：不要随意运行 arm.q.set()，仅做调试测试用，二关节的运动范围为(-3.1 -> 0)，三关节的范围为（1.6 -> -1.6）**

    > **请注意测试环境安全！！！固定底座后，检查碳管连接，人至少远离 1m 以上，附近不要站人**

    > **遥操作前，确保舵机校准零位，确保从臂的角度为 [0, -3.1, 1.6, 0, 0, 0, 0]**

    支持自动补全，主要命令：

    ```bash
    # 上电
    arm.enable()

    # 下电，无论处于 trace 模式还是重力补偿模式，都会先切换为pd模式，然后回到 rest 初始位置，再下电
    arm.disable()

    # 获取关节角度
    arm.q.get()

    # 设置关节角度，最后的0.5是速度
    arm.q.set(0, -3.1, 1.6, 0, 0, 0, 0, 0.5)

    # 运行到 7 字型位置
    arm.reset()

    # 恢复到 rest 初始位置
    arm.rest()

    # 切换为重力补偿模式
    arm.mode.set_gravity()
    ```

6. 测试通信示例

    ```bash
    # 测试 reset 和 disable
    python tests/test_reset_and_rest.py --port /dev/ttyACM0

    # 测试 pd 运动模式 move_q, 接连运动到设置的目标位置
    python tests/test_move_q.py --port /dev/ttyACM0

    # 测试轨迹追踪模式 stream q, 跟踪一小段轨迹，再 disable 下电恢复
    python tests/test_stream_q.py --port /dev/ttyACM0

    # 测试模式切换，切换到重力补偿模式时，可以拉到任意位置，disable 时先切换为 pd 运动模式，再运动到 rest 位置，再安全下电
    python tests/test_switch_mode.py --port /dev/ttyACM0
    ```

7. 测试拖动示教，默认50Hz

    ```bash
    # 开始录制轨迹，等待切换为重力补偿模式后，拖动机械臂，按esc退出
    python tests/test_record.py --port /dev/ttyACM0

    # 会打开保存的data.txt文件，进行跟踪
    python tests/test_move_q.py --port /dev/ttyACM0
    ```

8. 主臂校准

    ```bash
    # 首次运行会保存校准数据到 .follower_calibration 中
    lerobot-calibrate \
        --teleop.type=dm_arm_leader \
        --teleop.port=/dev/ttyUSB0 \
        --teleop.id=my_awesome_leader_arm \
        --teleop.fps=25
    ```

9.  测试遥操作

    ```bash
    # 不打开相机，控制频率 50 帧
    python tests/lerobot_telecontrol.py

    # 打开相机, 需要运行 find camera 找到相机 index
    # 控制频率 25 帧
    lerobot-teleoperate \
      --robot.type=dm_arm_follower \
      --robot.port=/dev/ttyACM0 \
      --robot.fps=25 \
      --robot.cameras="{
          wrist: {type: opencv, index_or_path: 8, width: 640, height: 480, fps: 30, rotation: 180},
          front: {type: opencv, index_or_path: 6, width: 640, height: 480, fps: 30}
        }" \
      --teleop.type=dm_arm_leader \
      --teleop.port=/dev/ttyUSB0 \
      --teleop.fps=25 \
      --display_data=true \
      --fps=25
    ```

10. 录数据集

    ```bash
    lerobot-record \
      --robot.type=dm_arm_follower \
      --robot.port=/dev/ttyACM0 \
      --robot.fps=25 \
      --robot.cameras="{
          wrist: {type: opencv, index_or_path: 9, width: 640, height: 480, fps: 30, rotation: 180},
          front: {type: opencv, index_or_path: 7, width: 640, height: 480, fps: 30}
        }" \
      --teleop.type=dm_arm_leader \
      --teleop.port=/dev/ttyUSB0 \
      --teleop.fps=25 \
      --display_data=true \
      --dataset.fps=25 \
      --dataset.repo_id=shenjianliang/DmArm \
      --dataset.push_to_hub=false \
      --dataset.num_episodes=50 \
      --dataset.episode_time_s=30 \
      --dataset.reset_time_s=15 \
      --dataset.single_task="Test" \
      --dataset.push_to_hub=true
    ```
