#!/bin/bash
set -e

sed -i 's/motor_platform_type: *7/motor_platform_type: 5/' src/jszr_mc/config/xg-user-parameters.yaml

sed -i 's/imu_type: *0/imu_type: 2/' src/jszr_mc/config/default-user-parameters.yaml
sed -i 's/motor_platform_type: *0/motor_platform_type: 1/' src/jszr_mc/config/default-user-parameters.yaml

sed -i 's/use_gamepad: *0/use_gamepad: 1/' src/jszr_mc/config/robot-defaults.yaml

sed -i 's/enable_stunts: *0/enable_stunts: 1/' src/jszr_mc/config/robot-defaults.yaml

echo "MC config modified"
