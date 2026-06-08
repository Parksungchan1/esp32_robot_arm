@echo off
cd C:\Users\parks\lerobot
.venv\Scripts\python lerobot/scripts/train.py ^
  --policy.type=act ^
  --dataset.repo_id=local/robot_arm ^
  --dataset.root=C:/Users/parks/esp32_robot_arm/lerobot_dataset ^
  --output_dir=C:/Users/parks/esp32_robot_arm/training_output ^
  --policy.chunk_size=50 ^
  --wandb.enable=false
