# Requirements & system reference

Hardware requirements, the exact software versions this workspace is verified
on, and GPU/CUDA setup for WSL and Docker. Verified **2026-05-29** on the
development machine.

> TL;DR: ROS 2 Humble + MoveIt 2 on WSL2/Ubuntu 22.04. **CUDA is optional** —
> the arm + RG6 pick-and-place runs fine on CPU. The machine *does* have an
> NVIDIA Quadro RTX 4000 (CUDA-capable) alongside an Intel UHD 630 (which does
> the RViz/OpenGL rendering via WSLg).

---

## 1. Hardware

### Robot cell (verified)
| Item | Detail |
|---|---|
| Arm | Universal Robots **UR10e** (e-Series), PolyScope **5.24.0.1219432**, S/N 20255201551 |
| Gripper | **OnRobot RG6** + **OnRobot Single Quick Changer** on the UR tool flange |
| Cabinet IP | `192.168.1.100` (cell subnet /24, direct Ethernet) |
| Laptop IP | `192.168.1.35` |
| Pendant URCaps | External Control; OnRobot; **rs485 daemon** (`rs485-1.0.jar`, for the ROS gripper) |
| Pendant Security | 5 services enabled (29999, 30001–30004, 50001–50004) + inbound **port 54321** allowed (gripper tool-comm) |

### Host PC (this machine)
| Item | This machine | Minimum / recommended |
|---|---|---|
| CPU | x86_64 | x86_64, 4+ cores |
| RAM | — | ≥ 16 GB recommended |
| GPU (compute) | **NVIDIA Quadro RTX 4000, 8 GB** | optional (only for CUDA workloads) |
| GPU (display/RViz) | **Intel UHD Graphics 630** (iGPU) | any WSLg-capable GPU |
| NIC | dedicated Ethernet to the robot cell | required for real hardware |

---

## 2. Software stack (verified versions)

| Component | This machine |
|---|---|
| Windows | 11 Pro, build 10.0.26200 |
| WSL | 2.7.3.0 |
| WSL kernel | 6.6.114.1-microsoft-standard-WSL2 |
| Linux distro | Ubuntu 22.04.5 LTS |
| ROS 2 | **Humble** (`/opt/ros/humble`) |
| MoveIt | 2 (Humble) + Pilz Industrial Motion Planner + OMPL |
| Python | 3.10.12 |
| Compiler | gcc 11.4.0 |
| Build tooling | colcon, vcstool, rosdep |
| pymodbus | 3.13.0 (RG6 Modbus client) |
| pyserial | 3.5 |
| socat | apt (`sudo apt install socat`) — tool-RS485 bridge |
| Networking | `~/.wslconfig` `networkingMode=mirrored` |

### ROS workspace dependencies
Vendor packages are pinned in [`ros2.repos`](ros2.repos) and imported with
`vcs import` (NOT committed here): `moveit2`, `Universal_Robots_ROS2_*`,
`ur_msgs`, `ur_client_library`, `onrobot1_ros`, `moveit_resources`. Build with
`colcon build --symlink-install`. See the README for the full bootstrap.

---

## 3. GPU / CUDA

### How it's wired on this machine (hybrid Intel + NVIDIA)
- **Intel UHD 630** → display + RViz/OpenGL, via WSLg **D3D12** passthrough
  (`OpenGL renderer: D3D12 (Intel(R) UHD Graphics 630)`). Details + verification:
  [`wiki/rviz_gpu_rendering.md`](wiki/rviz_gpu_rendering.md).
- **NVIDIA Quadro RTX 4000** → CUDA compute, exposed to WSL through `/dev/dxg`
  and `/usr/lib/wsl/lib/libcuda*` (the Windows driver passes through). `nvidia-smi`
  works inside WSL.

| GPU software | This machine |
|---|---|
| NVIDIA driver (Windows) | 573.91 (`32.0.15.7391`) |
| NVIDIA in WSL (`nvidia-smi`) | 570.205, CUDA **runtime 12.8** |
| CUDA **toolkit** (WSL) | **11.7.0** at `/usr/local/cuda` (`nvcc` present) |
| Intel iGPU driver (Windows) | 31.0.101.2137 |

**Important:** ROS 2 Humble + MoveIt + this pick-and-place do **not** require
CUDA — it's there for optional GPU compute (perception, ML, GPU-accelerated
nodes). RViz rendering uses the **Intel** iGPU, not the NVIDIA card.

### Enable CUDA in WSL
`nvidia-smi` already works (driver passthrough). The toolkit is installed but
`nvcc` is not on PATH — add it (e.g. to `~/.bashrc`):
```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
# verify:
nvidia-smi            # GPU + driver/runtime
nvcc --version        # toolkit (11.7 here)
```

### Fresh-machine CUDA-in-WSL setup (if not already installed)
1. Install a recent **NVIDIA Windows driver** with WSL support (Studio /
   Game-Ready / Quadro). This is what provides CUDA inside WSL —
   **do NOT install a Linux NVIDIA driver inside WSL.**
2. `wsl --update` (PowerShell), then confirm `nvidia-smi` works in WSL.
3. Install the **WSL-Ubuntu** CUDA toolkit (NOT the generic Linux runfile):
   ```bash
   wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.1-1_all.deb
   sudo dpkg -i cuda-keyring_1.1-1_all.deb && sudo apt-get update
   sudo apt-get install -y cuda-toolkit-11-7    # match your needs; 12.x also fine
   ```
4. Add to `PATH`/`LD_LIBRARY_PATH` as above.

### CUDA in Docker
GPU passthrough into containers needs the **NVIDIA Container Toolkit** on the
WSL host (Docker Desktop with the NVIDIA Windows driver usually wires this up
already):
```bash
# host (WSL) — if not already present:
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker

# verify GPU reaches a container:
docker run --rm --gpus all nvidia/cuda:11.7.1-base-ubuntu22.04 nvidia-smi
```
Run any GPU container with `--gpus all`, or in compose:
```yaml
services:
  my_gpu_node:
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
```
**Note:** RViz inside a container still renders through WSLg/D3D12 (Intel), not
the NVIDIA card — GPU passthrough here is for **compute**, not display.

---

## Last updated
2026-05-29 (verified on the dev machine: Win 11 / WSL 2.7.3 / Ubuntu 22.04.5,
ROS 2 Humble, NVIDIA RTX 4000 + CUDA 11.7, Intel UHD 630).
