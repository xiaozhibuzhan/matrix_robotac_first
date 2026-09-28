#!/bin/bash
set -e

# MATRiX Docker run script with GPU support
# This script launches the matrix-dev container with proper GPU, X11, and network setup

IMAGE_NAME="matrix-dev:latest"
CONTAINER_NAME="matrix-sim"

if [ "$#" -eq 0 ]; then
  CONTAINER_CMD=("./open_sim_launcher")
  echo "[INFO] No custom command provided. Defaulting to: ${CONTAINER_CMD[*]}"
else
  CONTAINER_CMD=("$@")
  echo "[INFO] Using custom container command: ${CONTAINER_CMD[*]}"
fi

echo "[INFO] Checking if container '$CONTAINER_NAME' already exists..."
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[INFO] Container exists. Stopping and removing old container..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
fi

echo "[INFO] Allowing X11 connections from local containers..."
xhost +local:root || echo "[WARN] xhost command failed - X11 forwarding may not work"

echo "[INFO] Starting container '$CONTAINER_NAME' with GPU support..."
docker run --gpus all -it --rm \
  --name "$CONTAINER_NAME" \
  --env="DISPLAY=${DISPLAY}" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --env="VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json" \
  --env="ROS_DOMAIN_ID=0" \
  --env="ROS_LOCALHOST_ONLY=0" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd):/workspace" \
  --volume="/usr/share/vulkan:/usr/share/vulkan:ro" \
  --volume="/usr/share/glvnd:/usr/share/glvnd:ro" \
  --device=/dev/dri \
  --device=/dev/input \
  --device=/dev/uinput \
  --volume=/run/udev:/run/udev:ro \
  --group-add=input \
  --volume=/dev/bus/usb:/dev/bus/usb \
  --network host \
  --privileged \
  --workdir=/workspace \
  "$IMAGE_NAME" \
  "${CONTAINER_CMD[@]}"

echo "[INFO] Container exited. Resetting X11 permissions..."
xhost -local:root 2>/dev/null || true
