# Networking from WSL2 → physical UR10e

A physical UR robot needs the ROS 2 driver to be on the *same Ethernet LAN*
as the controller. Vanilla WSL2 puts you behind a NAT and your container
cannot reach the robot. There are three real options, in order of how much
pain they save you.

## TL;DR — what to do on this machine

1. Make sure `~/.wslconfig` (Windows user profile) contains `networkingMode=mirrored`.
   The repo includes one at `C:\Users\libish m\.wslconfig` already.
2. From PowerShell: `wsl --shutdown`
3. Open a new WSL terminal: `wsl -d Ubuntu-22.04`
4. Verify: from WSL run `ip addr` — you should see your Windows Ethernet
   interfaces (Ethernet, Wi-Fi) listed, not just the old `eth0` NAT adapter.
5. From WSL: `ping 192.168.1.102` (or whatever your UR10e's IP is).

That's it. The container's `network_mode: host` now sees the WSL distro,
which mirrors Windows's NICs — so the container can speak to the UR
directly.

## Option A — Mirrored networking (recommended on Windows 11 + WSL >= 2.0)

**What it does:** WSL2 stops running its own private NAT'd network and
instead exposes Windows's network interfaces directly into the distro.
Containers running with `network_mode: host` see the same NICs Windows sees.

**Requirements**
- Windows 11 (you're on 10.0.26200, fine)
- WSL version 2.0 or later (you're on 2.7.3, fine)
- No need to install Hyper-V or change virtual switches.

**Configure** — put this in `%USERPROFILE%\.wslconfig` (already done):

```ini
[wsl2]
networkingMode=mirrored
localhostForwarding=true
dnsTunneling=true
firewall=true
```

**Apply** — Close every WSL window and run from PowerShell:

```powershell
wsl --shutdown
# wait ~5s, then open a new WSL terminal
```

**Verify**

```bash
ip route show       # should show Windows-style default gateways
ip addr             # should list Ethernet / Wi-Fi adapters, not the old eth0 NAT
```

The physical UR10e ping should now succeed:

```bash
ping -c 3 192.168.1.102
```

If a ROS 2 container is up with `network_mode: host`, this command from
inside the container has the same result:

```bash
docker compose -f docker/docker-compose.yml run --rm shell -- ping -c 3 192.168.1.102
```

## Option B — Hyper-V bridged switch (older Windows 10 / WSL 1.x fallback)

Use this only if mirrored mode is unavailable.

1. Install the Hyper-V feature (Windows Pro):
   ```powershell
   Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All
   ```
2. Reboot.
3. Open *Hyper-V Manager* → *Virtual Switch Manager* → New external switch
   bound to your physical Ethernet adapter, name it e.g. `WSLExternal`.
4. Edit `%USERPROFILE%\.wslconfig`:
   ```ini
   [wsl2]
   networkingMode=bridged
   vmSwitch=WSLExternal
   ```
5. `wsl --shutdown` then reopen.

Downsides: you lose Windows's seamless localhost forwarding; you may have to
manage WSL IP assignment manually; Wi-Fi switching can disconnect WSL.

## Option C — `netsh portproxy` (last resort)

If neither mirrored nor bridged is available (locked-down corporate machines,
etc.), you can forward specific TCP ports from Windows into WSL:

```powershell
# inbound: Windows -> WSL (for rosbridge etc.)
netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 \
   connectport=9090 connectaddress=$(wsl hostname -I | %{ $_.Trim().Split()[0] })
```

This is **one-way** and is fine for hosting `rosbridge`, but it doesn't help
the UR driver, which initiates outbound TCP/RTDE connections from the WSL
side. You cannot portproxy outbound; the UR will still be unreachable.

## Physical hookup (regardless of which option above)

- Dedicated Ethernet port on the PC, straight cable to the UR control box.
  Don't route through office routers or Wi-Fi — RTDE drops packets and the
  controller faults.
- Static IPs on the same subnet:
  - PC: `192.168.1.100`
  - UR10e: `192.168.1.102`
- On the UR teach pendant: *Settings → Network → DHCP off; Static IP*.
- From your shell, `ping 192.168.1.102` should answer in < 0.5 ms.

## Running the real-hardware driver in the container

Once mirrored networking is up and you can ping the robot from WSL:

```bash
cd ~/ur_rg6_ws
ROBOT_IP=192.168.1.102 docker compose -f docker/docker-compose.yml up ur_real
```

…or, in PowerShell, `.\docker\ur10e_rg6.ps1 real -RobotIp 192.168.1.102`.

On the UR teach pendant, load and **Play** the External Control URCap
program. Driver should print:

```
[ur_robot_driver] connected to UR at 192.168.1.102
[ur_robot_driver] received reverse interface handshake
```

If you instead see `connection refused` or `no route to host`, mirrored
networking did not take. Re-check `ip addr` inside WSL.

## Common gotchas

- **WSL still shows the old `eth0` NAT after editing `.wslconfig`.**
  You forgot `wsl --shutdown`. Any open shell holds the VM alive. Close
  Docker Desktop too, then `wsl --shutdown`, then re-open.
- **Mirrored mode breaks Docker Desktop port publishing.**
  It usually doesn't, but if `docker run -p 8080:80` stops forwarding,
  toggle Docker Desktop → Settings → Resources → Network → "Enable host
  networking" off then on.
- **VPN clobbers everything.** Most enterprise VPNs (Cisco AnyConnect,
  Global Protect) hijack the routing table and break mirrored mode. Either
  disconnect the VPN before driving the robot, or use a second NIC.
- **Robot's own firmware does not respond to ping.** Rare but possible —
  check the URCap is loaded and the controller is in "Remote Control" mode.

## Safety reminder

This entire stack now talks to a 25 kg, 1300 mm-reach industrial arm. Keep
your hand on the physical E-stop, work envelope cleared, speed slider at
≤ 20 % for first runs.
