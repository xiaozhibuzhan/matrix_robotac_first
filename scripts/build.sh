#!/bin/bash
set -e

bash scripts/install_deps.sh
# bash scripts/download_uesim.sh

# mc modify config and compile
# bash scripts/modify_config.sh
# bash scripts/build_mc.sh
# bash scripts/build_mujoco_sdk.sh
# bash scripts/build_navigo.sh

echo "Initialization complete."