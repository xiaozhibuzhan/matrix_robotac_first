#!/bin/bash
set -euo pipefail

echo "Downloading UeSim..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/../src"
UE_SIM_ZIP="UeSim.zip"
UE_SIM_DIR="UeSim"
GDRIVE_URL="https://drive.google.com/uc?id=1abu-Vi_l0UAx-ji4isJAvj8rkoG7jZ_i"

mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

if [ -d "$UE_SIM_DIR" ]; then
    echo "$UE_SIM_DIR already exists. Skipping download."
else
    gdown "$GDRIVE_URL" -O "$UE_SIM_ZIP"
    unzip -o "$UE_SIM_ZIP"
    rm "$UE_SIM_ZIP"
    chmod -R 755 "$UE_SIM_DIR"
    echo "UeSim downloaded and unpacked successfully."
fi