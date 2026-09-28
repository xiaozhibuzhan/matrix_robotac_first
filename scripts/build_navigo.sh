#! /bin/bash
set -e

cd  src/navigo

colcon build --packages-select pub_tf robots_dog_msgs

echo "navigo package complete."