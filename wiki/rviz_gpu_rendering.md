# RViz GPU rendering under WSL2

## Purpose

Document how RViz renders inside this WSL2 + WSLg setup, how to
verify it's hardware-accelerated, and what to change if it ever
falls back to software.

## Confirmed facts (verified 2026-05-24)

RViz on this machine renders through:

```
RViz (Ogre3D) → libGL (Mesa) → libGLX_mesa.so
              → libd3d12.so (in /usr/lib/wsl/lib/)
              → libdxcore.so (Direct3D in WSL)
              → /dev/dxg (WSL GPU passthrough kernel device)
              → Windows host's D3D12 driver
              → Intel UHD Graphics 630
```

**This is hardware-accelerated** — confirmed by:

```bash
$ glxinfo -B
direct rendering: Yes
Vendor: Microsoft Corporation (0xffffffff)
Device: D3D12 (Intel(R) UHD Graphics 630) (0xffffffff)
Accelerated: yes
Video memory: 32746MB
OpenGL renderer string: D3D12 (Intel(R) UHD Graphics 630)
```

Plus the RViz process maps the relevant libraries:

```bash
$ cat /proc/$(pgrep rviz2)/maps | awk '{print $6}' | sort -u \
    | grep -E 'libd3d12|libdxcore|libGL'
/usr/lib/wsl/lib/libd3d12.so
/usr/lib/wsl/lib/libd3d12core.so
/usr/lib/wsl/lib/libdxcore.so
/usr/lib/x86_64-linux-gnu/libGLX_mesa.so.0.0.0
/usr/lib/x86_64-linux-gnu/libGLdispatch.so.0.0.0
```

No software fallback (`llvmpipe`) loaded. The `Video memory: 32746MB`
figure is the Intel UHD's unified memory pool (shared with system
RAM); it's not actually 32 GB of dedicated VRAM — UHD 630 is an
integrated GPU.

## Why this matters

- **Smooth RViz updates at high planning-scene cardinality.** With
  ~10 attached/detached collision objects per pick-place cycle plus
  the live mesh-rich UR + RG6 robot model, software rendering chokes
  at single-digit FPS. D3D12 passthrough keeps it >30 FPS.
- **Glass surfaces, alpha blending, point clouds.** All cheap when
  GPU-rendered, expensive when llvmpipe.
- **No setup needed for this machine.** WSLg ships D3D12 passthrough
  out of the box on Windows 11.

## How to verify on a fresh machine

```bash
# 1. /dev/dxg must exist (WSL GPU device)
ls -la /dev/dxg

# 2. OpenGL renderer should NOT say "llvmpipe"
glxinfo -B | grep -i "OpenGL renderer"
# good:  OpenGL renderer string: D3D12 (Intel(R) UHD Graphics 630)
# good:  OpenGL renderer string: D3D12 (NVIDIA GeForce ...)
# BAD:   OpenGL renderer string: llvmpipe (LLVM 12.0.0, 256 bits)

# 3. Confirm direct rendering
glxinfo -B | grep "direct rendering"
# good: direct rendering: Yes
```

If `mesa-utils` is missing, install it:
```bash
sudo apt install mesa-utils
```

## If you see `llvmpipe` (software rendering)

That means WSL GPU passthrough isn't loading. Causes, in order of
likelihood:

1. **Wrong WSL version.** Run `wsl --version` (PowerShell). Need WSL
   2.0+ on Windows 11 (or recent WSL 1.x.x with Win 10). Update via
   `wsl --update`.
2. **Missing `/dev/dxg`.** If `ls /dev/dxg` returns "No such file or
   directory", the kernel doesn't have dxgkrnl. Update kernel:
   `wsl --update --pre-release`, then `wsl --shutdown` and reopen.
3. **GPU driver too old.** Windows-side: update Intel / NVIDIA / AMD
   driver to a version that supports WSL2. NVIDIA: 470.xx+, Intel:
   30.0.100+, AMD: Adrenalin 22.4+.
4. **DRI not on `LD_LIBRARY_PATH`.** WSLg's libs live at
   `/usr/lib/wsl/lib`. If something overrode that path (custom
   `LD_LIBRARY_PATH` env var), Mesa falls back to llvmpipe. Check
   `printenv LD_LIBRARY_PATH`; if non-empty, prepend `/usr/lib/wsl/lib`.
5. **Display socket missing.** `echo $DISPLAY` should be `:0` and
   `/mnt/wslg/` should exist. If not, WSLg isn't running — try
   `wsl --shutdown` from PowerShell and reopen.

## NVIDIA-specific note

If you later add an NVIDIA GPU, WSLg uses CUDA-enabled passthrough
through the same `/dev/dxg` route — no separate NVIDIA setup is
needed for RViz rendering. (CUDA compute in WSL is a separate
question: it needs `cuda-toolkit` inside Ubuntu plus a recent NVIDIA
Windows driver.)

## Last updated

2026-05-24.
