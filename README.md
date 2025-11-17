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
    git clone https://github.com/Jianliang-Shen/RobotARM.git
    cd RobotARM
    pip install -e .

    # additional packages
    pip install pyserial mujoco
    ```

3. 仿真测试

    ```bash
    python simulation/key_q_control.py

    python simulation/key_ee_control.py
    ```

4. 测试单臂(主臂)重力补偿: 假设主臂端口是 `/dev/ttyACM0`

    ```bash
    python test/master_arm_real2sim.py --port /dev/ttyACM0
    ```

5. 测试双臂遥操作: 假设从臂端口是 `/dev/ttyACM1`

    ```bash
    # 检查工作模式是 2（位置模式），夹爪工作模式是 4（力位混合模式）
    # 顺便检查关节位置是否是零位
    python test/follower_arm_read_param.py --port /dev/ttyACM1

    python test/test_double_arm.py

    # lerobot 遥操作
    lerobot-teleoperate \
      --robot.type=dm_arm_follower \
      --robot.port=/dev/ttyACM1 \
      --robot.cameras="{
          wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
          front: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}
        }" \
      --teleop.type=dm_arm_leader \
      --teleop.port=/dev/ttyACM0 \
      --display_data=true

    # record
    lerobot-record \
      --robot.type=dm_arm_follower \
      --robot.port=/dev/ttyACM1 \
      --robot.cameras="{
          wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
          front: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}
        }" \
      --teleop.type=dm_arm_leader \
      --teleop.port=/dev/ttyACM0 \
      --display_data=true \
      --dataset.repo_id=shenjianliang/test \
      --dataset.push_to_hub=false \
      --dataset.num_episodes=50 \
      --dataset.episode_time_s=30 \
      --dataset.reset_time_s=15 \
      --dataset.single_task="Test" \
      --dataset.push_to_hub=false
    ```

6. 舵机臂修改波特率

    ```bash
    # 运行后可能部分舵机没有设置成功，修改成功的返回 None
    # 断电，上电，重复运行脚本，直到返回 [None, None, None, None, None, None, None]
    python tool/uarm/change_baudrate.py

    # 运行 1000000 波特率的脚本，应当返回有效的位置信息
    python tool/uarm/test_new_baudrate.py
    ```

7. 舵机臂校准

    ```bash
    # 首次运行会保存校准数据到 .follower_calibration 中
    lerobot-calibrate \
        --teleop.type=servo_arm_leader \
        --teleop.port=/dev/ttyUSB0 \
        --teleop.id=my_awesome_leader_arm \
        --teleop.fps=25

    # 校准后运行，检查有没有偏差（仿真遥操作）
    python test/lerobot_telecontrol_uarm_mujoco.py
    ```

8. 测试舵机遥操作: 假设主臂端口是 `/dev/ttyUSB0`, 从臂是 `/dev/ttyACM0`

    ```bash
    # 不打开相机，控制频率 50 帧
    python test/lerobot_telecontrol_uarm.py

    # 打开相机, 需要运行 find camera 找到相机 index
    # 控制频率 30 帧
    lerobot-teleoperate \
        --robot.type=dm_arm_follower \
        --robot.port=/dev/ttyACM0 \
        --robot.cameras="{                                                           
            wrist: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30},
            front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}
          }" \
        --teleop.type=servo_arm_leader \
        --teleop.port=/dev/ttyUSB0 \
        --teleop.fps=25 \
        --display_data=true
    ```
