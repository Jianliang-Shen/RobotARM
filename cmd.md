
# ACT

```bash
lerobot-record --robot.type=dm_arm_follower --robot.port=COM18 --robot.cameras="{wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, front: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" --teleop.type=dm_arm_leader --teleop.port=COM13 --display_data=true --dataset.repo_id=shenjianliang/dm_arm_test --dataset.push_to_hub=false --dataset.num_episodes=20 --dataset.episode_time_s=30 --dataset.reset_time_s=15 --dataset.single_task="Pick the block" --dataset.push_to_hub=false

lerobot-train --dataset.repo_id=shenjianliang/dm_arm_test --policy.type=act --output_dir=outputs/train/act --job_name=act --policy.device=cuda  --wandb.enable=false --policy.repo_id=shenjianliang/act_policy


lerobot-record --robot.type=dm_arm_follower --robot.port=COM18  --robot.cameras="{wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, front: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" --display_data=true --dataset.repo_id=shenjianliang/eval_dm_arm_test --dataset.num_episodes=20 --dataset.single_task="Test" --policy.path=shenjianliang/act_policy
```

# SMOLVLA

```bash

pip install -e ".[smolvla]"

# Download smolvla model first

lerobot-train --policy.type=smolvla --policy.pretrained_path="D:\work\robot\RobotARM\smolvla_base" --dataset.repo_id=shenjianliang/dm_arm_test --batch_size=64 --steps=20000 --output_dir=outputs/train/my_smolvla --job_name=my_smolvla_training --policy.device=cuda --wandb.enable=false --policy.repo_id=shenjianliang/smolvla_policy

lerobot-record --robot.type=dm_arm_follower --robot.port=COM18 --robot.cameras="{wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, front: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" --dataset.single_task="Test" --dataset.repo_id=shenjianliang/eval_dm_arm_test --dataset.episode_time_s=50 --dataset.num_episodes=20 --policy.path=shenjianliang/smolvla_policy
```