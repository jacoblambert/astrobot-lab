#!/usr/bin/env bash
set -euo pipefail
IMAGE_TAG="isaac-sim-omnilrs:latest"

docker build \
  -f OmniLRS/omnilrs.docker/Dockerfile \
  -t "${IMAGE_TAG}" \
  OmniLRS/omnilrs.docker

echo "✅  Built ${IMAGE_TAG}"
