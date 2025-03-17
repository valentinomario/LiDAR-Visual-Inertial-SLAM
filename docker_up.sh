#!/bin/bash

rm -rf .env
rm -f compose.override.yaml

ARCH=$(uname -m)
USE_CUDA=true
BUILD_FLAG=""

for arg in "$@"; do
  if [ "$arg" == "--no-cuda" ]; then
    USE_CUDA=false
  elif [ "$arg" == "--build" ]; then
    BUILD_FLAG="--build"
  fi
done

if [ "$ARCH" = "aarch64" ]; then
  if [ "$USE_CUDA" = true ]; then
    echo 'IMAGE="valentinomario/lidar_visual_inertial_slam.cuda.aarch64:latest"' >> .env
  else
    echo 'IMAGE="valentinomario/lidar_visual_inertial_slam.aarch64:latest"' >> .env
  fi
else
  if [ "$USE_CUDA" = true ]; then
    echo 'IMAGE="valentinomario/lidar_visual_inertial_slam.cuda.x86_64"' >> .env
  else
    echo 'IMAGE="valentinomario/lidar_visual_inertial_slam.x86_64"' >> .env
  fi
fi

# echo 'USER_ID='$(id -u) >> .env
# echo 'GROUP_ID='$(id -g) >> .env


if [ -e /dev/video0 ]; then
  cat <<EOF > compose.override.yaml
services:
  lidar_visual_inertial_slam:
    devices:
      - /dev/video0
EOF
else
  echo "Device /dev/video0 not found. No video device will be mounted"
  cat <<EOF > compose.override.yaml
services:
  lidar_visual_inertial_slam:
    devices: []
EOF
fi

if [ "$USE_CUDA" == true ]; then
  cat <<EOF >> compose.override.yaml
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
EOF
fi

docker compose up -d $BUILD_FLAG

