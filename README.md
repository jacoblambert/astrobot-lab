# astrobot-lab

## Quick Start

### 1. Prep the repo

```bash
git clone <your-fork-or-repo-url>
cd astrobot-lab
git submodule update --init --recursive
```

If needed, adjust container UID/GID defaults in `.env`.

### 2. Build the Isaac Sim image

```bash
./docker/build_omnilrs_base.sh
```

This builds `isaac-sim-omnilrs:latest` from `OmniLRS/omnilrs.docker/Dockerfile`.

### 3. Run on a remote RTX machine (recommended)

Use the remote compose stack on the GPU host:

```bash
docker compose -f docker-compose.remote.yml up -d
docker compose -f docker-compose.remote.yml logs -f astrobot-sim
```

Container roles:
- `astrobot-sim`: Isaac Sim server (headless livestream by default).
- `astrobot-dev`: Space ROS dev shell on the same host/network.

Open a shell in the dev container:

```bash
docker compose -f docker-compose.remote.yml exec astrobot-dev bash
```

Switch the sim service to launch OmniLRS directly:

```bash
SIM_MODE=omnilrs docker compose -f docker-compose.remote.yml up -d --force-recreate astrobot-sim
```

### 4. Optional local compose (X11-based)

For a local GPU + X11 workflow:

```bash
docker compose up -d astrobot-sim astrobot-dev
```

The remote workflow is preferred when your local machine does not have a supported RTX GPU.
