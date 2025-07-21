#!/usr/bin/env bash
set -euo pipefail
IMAGE_TAG="omnilrs:4.5.0-humble"

docker build \
  -f OmniLRS/omnilrs.docker/Dockerfile \
  -t "${IMAGE_TAG}" \
  OmniLRS/omnilrs.docker

echo "✅  Built ${IMAGE_TAG}"