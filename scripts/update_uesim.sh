#!/bin/bash

# 设置参数
FOLDER_ID="1wHmmBYj8xNIu-n_0Ivs0FmRHDb8GXzsN"
TARGET_DIR="src/UeSim"
TMP_DIR="./UeSim"
ZIP_NAME="Linux.zip"
TAR_NAME="Linux.tar.gz"

# 下载整个文件夹（需要gdown >=4.6.0 支持文件夹）
echo "[INFO] 正在使用gdown下载UeSim文件夹..."
gdown --folder "https://drive.google.com/drive/folders/${FOLDER_ID}" -O "$TMP_DIR"

# 判断压缩包是否存在
cd "$TMP_DIR" || { echo "[ERROR] 无法进入 $TMP_DIR"; exit 1; }

if [ -f "$ZIP_NAME" ]; then
    echo "[INFO] 找到 $ZIP_NAME，开始解压..."
    unzip -q "$ZIP_NAME" -d Linux
    rm "$ZIP_NAME"
elif [ -f "$TAR_NAME" ]; then
    echo "[INFO] 找到 $TAR_NAME，开始解压..."
    mkdir Linux && tar -xzf "$TAR_NAME" -C Linux
    rm "$TAR_NAME"
else
    echo "[ERROR] 没有找到名为 Linux 的压缩包"
    exit 1
fi

# 回到主目录
cd ..

# 创建目标路径
mkdir -p "$TARGET_DIR"

# 拷贝覆盖
echo "[INFO] 正在复制 UeSim/Linux 到 $TARGET_DIR..."
cp -rf "$TMP_DIR/Linux/"* "$TARGET_DIR"

# 清理缓存
echo "[INFO] 正在删除临时目录 $TMP_DIR"
rm -rf "$TMP_DIR"

echo "[✅ DONE] UeSim 更新完成！"

