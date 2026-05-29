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
5. From WSL: `ping 192.168.1.100` (or whatever your UR10e's IP is).

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
ping -c 3 192.168.1.100
```

If a ROS 2 container is up with `network_mode: host`, this command from
inside the container has the same result:

```bash
docker compose -f docker/docker-compose.yml run --rm shell -- ping -c 3 192.168.1.100
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
- Static IPs on the same subnet (VERIFIED cell config — do not invent others):
  - PC (laptop): `192.168.1.35`
  - UR10e cabinet: `192.168.1.100`
- On the UR teach pendant: *Settings → Network → DHCP off; Static IP*.
- From your shell, `ping 192.168.1.100` should answer in < 0.5 ms.

## UR cabinet ports + the RG6 gripper over RS485 (2026-05-28)

Beyond plain reachability, the cabinet's own **Security firewall** gates which
TCP ports answer. Two things must be set on the pendant
(*Settings → Security*), or you'll get `connection refused` despite a good ping:

- **Enable all 5 robot services** (29999 dashboard, 30001–30004 primary/RTDE,
  + the 50001–50004 external-control ports). They ship DISABLED.
- **Inbound port allow-list** (*Security → General*): non-standard ports are
  blocked. The **RG6 gripper tool-communication uses port `54321`**, so it must
  be ADDED to the allowed inbound ports — otherwise the host can't reach the
  rs485 daemon even though it's listening. (We changed the disabled range to
  `1-21,23-54320,54322-65535`, i.e. allow 22 + 54321.)

**How the gripper talks (RS485/Modbus, NOT the old URScript path):** the RG6 is
driven over the UR **tool-flange RS485**. `ur_robot_driver` with
`use_tool_communication:=true` bridges that to a host pseudo-serial device
(`/tmp/ttyUR`) via **socat**; a Modbus-RTU client (`tests/onrobot_modbus_grip.py`)
reads/writes the gripper registers. Requires on the cabinet: the **rs485 daemon
URCap** (`/root/.urcaps/rs485-1.0.jar`, coexists with the OnRobot URCap), the
**`ros` installation** (Tool I/O Controlled by User + Communication Interface +
24 V + OnRobot device None), and **`external_control.urp` PLAYING** (the bridge
runs inside the control program). Full detail: `../wiki/rg6_rs485_modbus.md`.

> **Use the WSL-native scripts for hardware**, not the Docker container: the
> gripper bridge needs `socat` + `use_tool_communication` + the cabinet rs485
> URCap, which the container path below does NOT wire up. Run
> `bash scripts/launch_real_rs485.sh` then `bash scripts/grip.sh {open|close}`
> / `bash scripts/play_pickplace.sh --max 4 --real-gripper` from a normal
> (non-admin) WSL terminal. See the README command cheat-sheet.

## Running the real-hardware driver in the container

Once mirrored networking is up and you can ping the robot from WSL:

```bash
cd ~/ur_rg6_ws
ROBOT_IP=192.168.1.100 docker compose -f docker/docker-compose.yml up ur_real
```

…or, in PowerShell, `.\docker\ur10e_rg6.ps1 real -RobotIp 192.168.1.100`.

On the UR teach pendant, load and **Play** the External Control URCap
program. Driver should print:

```
[ur_robot_driver] connected to UR at 192.168.1.100
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
