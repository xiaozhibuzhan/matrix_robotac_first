#!/bin/bash
set -e

cd src/jszr_mujoco/simulate
cmake -S . -B build
cmake --build build -j32

echo "Mujoco SDK build complete."