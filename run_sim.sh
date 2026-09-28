#!/usr/bin/env bash
set -euo pipefail

#######################################
# 基础
#######################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ROBOT_ARG="${1:-xgb}"
SCENE_ID="${2:-1}"
OFFSCREEN="${3:-0}"
PIXELSTREAM="${4:-0}"
MUJOCORUNNING="${5:-0}"

#######################################
# 防止并发启动（文件锁）
#######################################
LOCKFILE="/tmp/run_sim_$(id -u).lock"
exec 9>"$LOCKFILE"
if ! flock -w 60 9; then
    echo "[ERROR] 无法获取启动锁，请稍后重试"
    exit 1
fi
echo "[INFO] 启动锁已获取 (PID $$)"

#######################################
# 全局 PID 管理
#######################################

pkill -TERM -f robot_mujoco || true
pkill -TERM -f jszr_mujoco_ue || true
pkill -TERM -f UnrealGame || true
pkill -TERM -f UE4Editor || true
pkill -TERM -f mc_ctrl || true

# 等待旧进程完全退出，避免 cleanup 钩子与新实例互相干扰
sleep 3


PIDS=()
CLEANUP_DONE=0  # 防止 EXIT + SIGTERM 双重触发

# 等待指定 PID 退出，超时后强制 SIGKILL（只操作自己启动的 PID，不按名字盲杀）
_wait_or_kill() {
    local pid=$1 timeout=${2:-5}
    [[ -z "$pid" ]] && return
    kill -0 "$pid" 2>/dev/null || return  # 已经不存在
    kill -TERM "$pid" 2>/dev/null || return
    local i=0
    while (( i < timeout )); do
        sleep 1
        kill -0 "$pid" 2>/dev/null || return  # 已正常退出
        (( i++ ))
    done
    echo "[WARN] PID $pid 未在 ${timeout}s 内退出，强制 SIGKILL"
    kill -9 "$pid" 2>/dev/null || true
}

cleanup() {
    [[ "$CLEANUP_DONE" == "1" ]] && return
    CLEANUP_DONE=1
    echo "[INFO] ===== Cleaning up processes ====="

    # 只按 PID 精确关闭本脚本启动的进程，避免误杀新实例
    for pid in "${PIDS[@]:-}"; do
        if [[ -n "$pid" ]]; then
            echo "[INFO] 关闭 PID $pid"
            _wait_or_kill "$pid" 5
        fi
    done

    echo "[INFO] ===== Cleanup finished ====="
    # 释放文件锁，允许下一次启动
    flock -u 9 2>/dev/null || true
}
trap cleanup EXIT SIGINT SIGTERM

#######################################
# Offscreen / PixelStreaming
#######################################
USE_OFFSCREEN=""
[[ "$OFFSCREEN" == "1" ]] && USE_OFFSCREEN="-RenderOffScreen"

USE_PIXELSTREAMER=""
[[ "$PIXELSTREAM" == "1" ]] && USE_PIXELSTREAMER="-PixelStreamingURL=ws://127.0.0.1:8888"

#######################################
# 场景配置
#######################################
SCENE="unreal.xml"
MAPNAME="/Game/Maps/RobotAC/RobotAC_Level_1"
WEAPON=""

case "$SCENE_ID" in
    0)  SCENE="unreal.xml"; MAPNAME="/Game/Maps/RobotAC/RobotAC_Level_2" ;;
    1)  SCENE="unreal.xml";     MAPNAME="/Game/Maps/RobotAC/RobotAC_Level_2" ;;
    2)  SCENE="unreal.xml";    MAPNAME="/Game/Maps/RobotAC/RobotAC_Level_2" ;;
    *)
        echo "[WARN] Unknown scene id $SCENE_ID, using default"
        ;;
esac

sed -i "s/^robot_scene: .*/robot_scene: \"$SCENE\"/" src/robot_mujoco/simulate/config.yaml

#######################################
# 机器人类型 & 启动策略
#######################################
TARGET_FILE="src/robot_mc/run_mc.sh"
ENABLE_MUJOCO=false
ENABLE_MC=false
ROBOTTYPE="xgb"

# MUJOCORUNNING is 1 config/config.json中"mujoco_running": true，否则为 false
if [[ "$MUJOCORUNNING" == "1" ]]; then
    ENABLE_MUJOCO=true
    sed -i "s/\"mujoco_running\": .*/\"mujoco_running\": true,/" config/config.json
    echo "[INFO] MuJoCo will be enabled. Please ensure you have the proper license and setup."
else
    ENABLE_MUJOCO=false
    sed -i "s/\"mujoco_running\": .*/\"mujoco_running\": false,/" config/config.json
    echo "[INFO] MuJoCo will be disabled. The simulation will run without physics-based dynamics."
fi


case "$ROBOT_ARG" in
    1|xgb)
        ROBOTTYPE="xgb"
        ENABLE_MC=true
        sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XG/' "$TARGET_FILE"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xg-user-parameters.yaml
        else
            ENABLE_MUJOCO=false
            sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xg-user-parameters.yaml
        fi
        ;;
    *)
        echo "[ERROR] Unknown robot type: $ROBOT_ARG"
        exit 1
        ;;
esac

sed -i "s/^robot: .*/robot: \"$ROBOTTYPE\"/" src/robot_mujoco/simulate/config.yaml

#######################################
# JSON 同步
#######################################
jq ".robot.robot_type=\"$ROBOTTYPE\" | .robot.weapon=\"$WEAPON\"" \
    config/config.json > /tmp/config.json && mv /tmp/config.json config/config.json

cp config/config.json src/UeSim/Linux/UeSim/Content/model/config/config.json
cp src/UeSim/Linux/UeSim/Content/model/xgb/unreal.xml src/robot_mujoco/zsibot_robots/xgb/unreal.xml

#######################################
# 机器人初始位姿
#######################################
ROBOT_X=$(jq -r '.robot.position.x' config/config.json)
ROBOT_Y=$(jq -r '.robot.position.y' config/config.json)
XML_FILE="src/robot_mujoco/zsibot_robots/${ROBOTTYPE}/${ROBOTTYPE}.xml"

sed -i "s/<body name=\"base_link\" pos=\"[^\"]*\"/<body name=\"base_link\" pos=\"${ROBOT_X} ${ROBOT_Y} 0.65\"/" "$XML_FILE"

#######################################
# 启动流程
#######################################
echo "[INFO] Starting processes..."




cd src/UeSim/Linux
echo "[INFO] Starting UE"
./UeSim.sh -game -LogCmds="global off" -ExecCmds="t.MaxFPS 30" $USE_OFFSCREEN $USE_PIXELSTREAMER > UeSim.log 2>&1 &
PIDS+=($!)

sleep 5

cd ../../robot_mc
if $ENABLE_MC; then
    echo "[INFO] Starting MC"
    ./run_mc.sh r mc_enable=true > run_mc.log 2>&1 &
    PIDS+=($!)
fi

cd ../robot_mujoco/simulate/build
if $ENABLE_MUJOCO; then
    echo "[INFO] Starting MuJoCo"
    ./robot_mujoco > robot_mujoco.log 2>&1 &
    PIDS+=($!)
fi
#######################################
# 阻塞等待
#######################################
echo "[INFO] All components started."
wait
