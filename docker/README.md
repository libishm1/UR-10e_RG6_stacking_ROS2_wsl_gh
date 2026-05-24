# UR10e + RG6 — Docker package

Builds and runs this workspace inside a ROS 2 Humble container.
Targeted at **Windows + Docker Desktop + WSL2 (Ubuntu-22.04)**.

Two images are provided:

| Image / Dockerfile        | What it contains                                                                 | When to use                                                                |
|---------------------------|----------------------------------------------------------------------------------|----------------------------------------------------------------------------|
| `ur10e_rg6:humble` (`Dockerfile`)       | apt deps + entrypoint; **workspace mounted from host**                  | Active development — edit xacro/configs from VS Code, container rebuilds.  |
| `ur10e_rg6:full`   (`Dockerfile.full`)  | apt deps **+ git clone of the repo + `vcs import` of vendors + colcon build** baked in | Reproducible install on any Docker host — `docker run` is all you need.    |

---

## One-time host setup

1. Install **Docker Desktop for Windows** with the WSL2 backend enabled.
2. In Docker Desktop → Settings → Resources → WSL Integration, enable `Ubuntu-22.04`.
3. Confirm from a WSL shell: `docker version`.
4. If you are on Windows 11 + WSL2, **WSLg already handles X11**; no VcXsrv needed.
   The compose file mounts `/mnt/wslg` so RViz windows show up automatically.

## Build the image

### A. Self-contained (recommended for first-time install)

Builds an image with everything baked in — no host workspace needed at run-time.
Takes ~15-20 min on first build (downloads vendor packages, runs `colcon build`).

```bash
# In WSL
docker compose -f docker/docker-compose.yml --profile build_only build ur10e_rg6_full
# or with a different fork / branch:
docker build -f docker/Dockerfile.full \
    --build-arg REPO_URL=https://github.com/libishm1/UR-10e_RG6_stacking_ROS2_wsl_gh.git \
    --build-arg REPO_REF=main \
    -t ur10e_rg6:full .
```

Then launch the full MoveIt 2 stack (fake hardware):
```bash
docker compose -f docker/docker-compose.yml up full_stack
# or against a real UR10e:
USE_FAKE_HARDWARE=false ROBOT_IP=192.168.1.102 \
    docker compose -f docker/docker-compose.yml up full_stack
```

Run the pick-and-place demo against the same container (separate shell):
```bash
docker exec -it ur10e_rg6_full python3 /workspace/tests/play_pickplace.py
```

### B. Dev image (workspace mounted from host)

Use this when you're iterating on the xacro/configs locally:
```bash
docker compose -f docker/docker-compose.yml --profile build_only build ur10e_rg6
```

Or from PowerShell:
```powershell
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/ur_rg6_ws && docker compose -f docker/docker-compose.yml --profile build_only build ur10e_rg6"
```

## Run things

| Service     | Image used               | What it does                                                    |
|-------------|--------------------------|-----------------------------------------------------------------|
| full_stack  | `ur10e_rg6:full`         | Self-contained MoveIt 2 + UR driver + RViz (fake or real HW)    |
| rviz_view   | `ur10e_rg6:humble` (dev) | RViz with joint sliders                                         |
| ur_fake     | `ur10e_rg6:humble` (dev) | Full UR driver + controllers, simulated hardware                |
| ur_real     | `ur10e_rg6:humble` (dev) | Full UR driver against a physical UR10e (set `ROBOT_IP`)        |
| rosbridge   | `ur10e_rg6:humble` (dev) | Exposes ROS 2 on `ws://localhost:9090` for Grasshopper/roslibpy |
| shell       | `ur10e_rg6:humble` (dev) | Interactive bash inside the container                           |

Launch any of them with `docker compose -f docker/docker-compose.yml up <service>`.

The whole workspace at `~/ur_rg6_ws` is bind-mounted to `/workspace` inside the container, so edits to your xacro/configs from VS Code (Remote-WSL) are live.

## First-run build behaviour

The entrypoint runs `colcon build --symlink-install` automatically on the first start when no `install/setup.bash` exists. Skip MoveIt-from-source so the initial build is fast (MoveIt is pulled in via apt). To force a rebuild later:

```bash
FORCE_BUILD=1 docker compose -f docker/docker-compose.yml up ur_fake
```

To skip the auto-build entirely:

```bash
SKIP_BUILD=1 docker compose -f docker/docker-compose.yml run --rm shell
```

## Connecting Rhino/Grasshopper

1. `docker compose -f docker/docker-compose.yml up rosbridge`
2. In Grasshopper (roslibpy) connect to `ws://localhost:9090` (WSL2 forwards localhost to the distro).
   If that fails, grab the distro IP from PowerShell: `wsl hostname -I` and use that.

## Common gotchas

- **RViz window doesn't appear.** Check `echo $DISPLAY` inside `docker compose run --rm shell` — should be `:0`. If empty, your host shell didn't have `DISPLAY` set; export it first or run `xeyes` to verify WSLg is working.
- **Port 9090 already taken** by something else on Windows. Stop the conflicting process or change `network_mode: host` to a forwarded port for the `rosbridge` service.
- **No `install/setup.bash` after first start.** The build may have failed silently. Run `docker compose run --rm shell`, then `cd /workspace && colcon build` manually and read the errors.
- **`ur_real` cannot reach the UR10e.** With `network_mode: host` on WSL2 the container sees the *WSL* network, not the Windows LAN. You may need to set up a Hyper-V bridge or connect the UR to the WSL adapter. For Windows hosts the cleaner option is to run the real-hardware driver natively on a Linux host instead of through WSL.

## File layout

```
~/ur_rg6_ws/
├── docker/
│   ├── Dockerfile          ← dev image (workspace mounted from host)
│   ├── Dockerfile.full     ← self-contained image (repo + vcs import + build)
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   ├── .dockerignore
│   └── README.md           ← you are here
├── ros2.repos              ← consumed by Dockerfile.full via `vcs import`
├── src/                    ← bind-mounted into /workspace/src for dev image
├── install/                ← regenerated by entrypoint colcon build (dev image only)
└── ...
```
