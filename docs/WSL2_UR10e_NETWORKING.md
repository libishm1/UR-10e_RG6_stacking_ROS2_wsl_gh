# WSL2 ↔ UR10e — networking deep-dive + fallback ladder

This guide consolidates everything we know about getting the
`ur_robot_driver` running inside WSL2 (Ubuntu 22.04) talking to a
real UR10e on the LAN. It is built from:

1. The verified cell config in `D:\robot_ws\robots\outputs\2026-05-09\SESSION_CLOSE.md`.
2. `espenakk/ros2-wsl2-guide` (Hyper-V vSwitch + bridged WSL2 recipe).
3. ROS Answers question 413680 — "ROS2 + WSL2 external communication".
4. YouTube tutorial `NOOUfsExYCE` (referenced by 413680 — direct
   hardware NIC access for WSL2).
5. `microsoft/WSL` discussions #10614, #9227 (mirrored multicast loss, bridged failures).
6. UR ROS 2 driver docs + `Universal_Robots_ExternalControl_URCap` issue #24
   (the canonical WSL2 reverse-socket failure).
7. `randombytes.substack.com/bridged-networking-under-wsl`.

## What the UR ROS 2 driver actually needs over the network

| Direction | Protocol | Port | Initiator | Reason |
|---|---|---|---|---|
| WSL → cabinet | TCP | 29999 | driver / dashboard | program load / play / state |
| WSL → cabinet | TCP | 30001 | (rare) | primary client interface |
| WSL → cabinet | TCP | 30002 | driver | secondary client (URScript stream) |
| WSL → cabinet | TCP | 30004 | driver | RTDE state / control |
| **Cabinet → WSL** | **TCP** | **50001 + 50002** | **External Control URCap (after Play)** | **reverse channel — driver listens, robot connects out** |
| WSL ↔ WSL nodes | UDP | 7400-7600 | DDS | discovery + data (multi-machine) |

**The critical row is the reverse one.** When you press Play on the
External Control program, the cabinet opens an outbound TCP connection
to the Host IP configured in the URCap on ports 50001 (script) +
50002 (trajectory). That IP must be **routable from the cabinet to
the listener inside WSL2**. Every WSL2 networking gotcha below comes
down to whether this reverse connection lands or not.

Multicast / DDS only matters if you run ROS 2 nodes on another machine
on the LAN (e.g. a remote RViz). The UR driver itself uses unicast
TCP only — so multicast loss in mirrored mode does NOT break the
driver.

## Fallback ladder (most-to-least preferred for this cell)

### Level 0 — verify Windows + cabinet basics (do this first, every session)

From PowerShell on Windows:
```powershell
# Cabinet reachable from Windows host?
ping 192.168.1.100
# Cabinet's TCP services up?
Test-NetConnection 192.168.1.100 -Port 29999
Test-NetConnection 192.168.1.100 -Port 30004
```
If `Test-NetConnection` times out (TcpTestSucceeded: False) → pendant
side is the problem. Go to "Pendant prereqs (user-only tasks)" below.
Don't touch WSL until ping + TCP probe both work from PowerShell.

### Level 1 — mirrored mode (current setup, simplest)

This is what `~/.wslconfig` already has. Best when:
- The UR is the only thing WSL needs to talk to (no remote ROS 2 nodes)
- You're on Windows 11 Home (Hyper-V unavailable for Level 2)

`~/.wslconfig` (Windows side):
```ini
[wsl2]
networkingMode=mirrored
firewall=false
hostAddressLoopback=true
```

`/etc/wsl.conf` (WSL side, optional):
```ini
[boot]
systemd=true
[network]
generateResolvConf=true
```

After any `.wslconfig` change:
```powershell
wsl --shutdown
# wait 5 s
wsl -d Ubuntu-22.04
```

**URCap "Host IP" field on the pendant:** enter the **Windows host's
LAN IP** (`192.168.1.35` for this cell). Mirrored mode makes the WSL
VM share the host's network stack, so the cabinet's reverse TCP to
`192.168.1.35:50002` lands on the listener inside WSL.

**Windows Firewall rule for the reverse channel (one-time, PowerShell as admin):**
```powershell
New-NetFirewallRule -DisplayName "UR External Control 50001-50002" `
    -Direction Inbound -Action Allow -Protocol TCP `
    -LocalPort 50001-50002
# Optional: ROS 2 DDS discovery (only if remote ROS 2 nodes)
New-NetFirewallRule -DisplayName "ROS2 DDS UDP 7400-7600" `
    -Direction Inbound -Action Allow -Protocol UDP `
    -LocalPort 7400-7600
# Mirrored-mode-specific Hyper-V firewall override (per microsoft/WSL #10614)
Set-NetFirewallHyperVVMSetting `
    -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
    -DefaultInboundAction Allow
```

**Verify from WSL:**
```bash
# Cabinet reachable?
ping -c 3 192.168.1.100
# RTDE port open?
nc -vz 192.168.1.100 30004
# What IP does WSL think it has? (mirrored mode shows the host's)
ip -4 addr show eth0 | grep inet
```

**Known issues (mirrored):**
- UDP multicast **receive** broken (microsoft/WSL #10614) — WSL2 can
  send multicast but can't receive it. Irrelevant for UR; relevant if
  you want remote ROS 2 nodes to discover the driver. Fix: switch to
  Level 2 (bridged) or run all ROS 2 nodes inside the same WSL.
- Cabinet sometimes connects but driver logs "Connection to reverse
  interface dropped" repeatedly. Almost always the OnRobot URCap
  cold-boot quirk (see "Cabinet quirks" below), NOT the network.

### Level 2 — bridged mode (recommended for Windows 11 Pro)

Required when: you need full DDS multicast (remote ROS 2 nodes), OR
mirrored mode failed for a hard-to-debug reason, OR you want WSL on
its own LAN IP.

**Prereqs (Windows 11 Pro only — Hyper-V is not on Home):**
```powershell
# Verify Hyper-V is enabled
Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All
# Enable if Disabled:
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All -All
# (reboot)
```

**Create a Hyper-V external switch bound to your Ethernet adapter:**
```powershell
# Find your adapter name (the one on the same LAN as the UR)
Get-NetAdapter | Format-Table Name, InterfaceDescription, Status, LinkSpeed
# Create external switch (replace "Ethernet" with the name from above)
New-VMSwitch -Name "External Switch" -NetAdapterName "Ethernet" -AllowManagementOS $true
```
Windows briefly drops connectivity while the switch is created. After
~30 s the Ethernet adapter shows up as bound to the external switch
in Network Connections.

**`~/.wslconfig` (Windows side):**
```ini
[wsl2]
networkingMode=bridged
vmSwitch=External Switch
firewall=false
```
(Replace `External Switch` if you used a different name.)

```powershell
wsl --shutdown
wsl -d Ubuntu-22.04
ip -4 addr show eth0    # eth0 now has its own LAN IP, e.g. 192.168.1.42
```

**URCap "Host IP" field on the pendant:** enter the **WSL VM's LAN IP**
(`192.168.1.42` in the example above — whatever `ip addr` shows). The
cabinet now reaches WSL directly without going through the Windows
stack. No port-forwarding, no firewall override needed for the reverse
channel.

**Known issues (bridged):**
- WSL VM gets a DHCP lease — IP can change across `wsl --shutdown`
  cycles. Either set a DHCP reservation on the router for the WSL VM's
  MAC, OR re-enter the URCap Host IP after each WSL restart.
- Windows 11 Pro only. Bridged on Home will silently fail to start the
  VM (microsoft/WSL #9227).
- VPN clients on Windows do not flow through to the bridged WSL VM.
  If your cell is behind a VPN, stay on Level 1 (mirrored) or install
  the VPN client inside WSL.

### Level 3 — default NAT + port forwarding (fallback if 1+2 fail)

WSL2's default NAT mode gives the VM a `172.x.y.z` IP that nothing on
the LAN can reach. Port forwarding from the Windows host to WSL bridges
the reverse channel.

**Find the WSL VM IP (Windows side):**
```powershell
wsl -d Ubuntu-22.04 -- hostname -I
# e.g. 172.19.x.y
$wslIP = (wsl -d Ubuntu-22.04 -- hostname -I).Trim().Split()[0]
```

**Set up `netsh` port-proxy:**
```powershell
# Forward from Windows host's LAN IP (192.168.1.35) into WSL on 50001 and 50002
netsh interface portproxy add v4tov4 listenport=50001 listenaddress=192.168.1.35 connectport=50001 connectaddress=$wslIP
netsh interface portproxy add v4tov4 listenport=50002 listenaddress=192.168.1.35 connectport=50002 connectaddress=$wslIP
netsh interface portproxy show all
# Allow the listen ports through firewall (same rule as Level 1)
New-NetFirewallRule -DisplayName "UR External Control 50001-50002" `
    -Direction Inbound -Action Allow -Protocol TCP `
    -LocalPort 50001-50002
```

**URCap "Host IP" field:** Windows host LAN IP (`192.168.1.35`). The
cabinet sees a normal LAN host; the portproxy invisibly relays into
WSL.

**Known issues (NAT + portproxy):**
- WSL VM IP changes on every `wsl --shutdown` → port-proxy entries
  become stale. Either rebuild them on every boot (Task Scheduler
  script), or live with re-running the netsh commands.
- Easy to forget the firewall rule and waste an hour debugging.
- The driver's RTDE outbound to the cabinet works fine (NAT outbound
  is unrestricted); only the reverse channel needs the portproxy.

### Level 4 — native Linux box on the same LAN

If WSL2 keeps fighting you, the lowest-friction fallback is to take
the workspace to a real Ubuntu 22.04 machine (laptop or a NUC) and
plug it into the cell switch. No WSL gotchas at all. Use this when
running the cell in production.

## Cabinet-side prerequisites (USER-ONLY TASKS)

These cannot be done from Claude — you have to be at the pendant.

### 1. Enable cabinet TCP services (one-time, but resets on factory reset)

From the pendant: `Settings → Security → enable Services`. Toggles
29999 / 30001 / 30002 / 30003 / 30004 from blocked to allowed. Without
this, every TCP connection from WSL times out and looks like a network
problem — but it's a PolyScope-level firewall. Discovered the hard way
on 2026-05-09 (D:\robot_ws SESSION_CLOSE.md).

### 2. Allow port 22 (SSH) so the Path B fallback works

`Settings → Security → General → Disable inbound access to additional
interfaces (by port)` → change `1-65535` to `1-21,23-65535`. Excludes
port 22 from the blocklist.

### 3. Install SSH key on pendant (one-time)

The existing key at `D:\robot_ws\robots\outputs\2026-05-09\ssh_setup\robots_workspace_key.pub`
is already enrolled as `robots-workspace-2026-05-10`. Re-import via
USB if the cabinet was factory-reset:

```
Settings → Security → Secure Shell → Unlock (Admin pw)
   → Enable SSH access: ON
   → Authentication: Both
   → Manage Authorized Keys → Add → import from USB
```

### 4. Set cabinet to Remote Control mode

Top-right toggle on the pendant. Required for the External Control
URCap to accept commands.

### 5. Install the External Control URCap

Already on this pendant. If reinstalled:
- Download from https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases
- USB → pendant → Settings → System → URCaps → +
- Restart cabinet, accept the EULA on first run.

### 6. Create the External Control program on the pendant

Program tab → URCaps → External Control → drag into program tree.
Then in its config:

- **Host IP**: depends on networking level above
  - Level 1 (mirrored): Windows host IP (`192.168.1.35`)
  - Level 2 (bridged): WSL VM IP (`192.168.1.42` or whatever)
  - Level 3 (NAT + portproxy): Windows host IP (`192.168.1.35`)
- **Host Name**: leave blank
- **Custom Port**: 50002 (default — leave as-is)

Save as `external_control.urp`. Don't press Play yet; the driver
launches first.

### 7. Network on the pendant

`Settings → System → Network`:
- Static IP `192.168.1.100`
- Subnet `255.255.255.0`
- Gateway empty (we're on a direct point-to-point cable)
- DNS empty
- Apply, wait for "Network is connected: GREEN ✓".

### 8. OnRobot URCap cold-boot quirk (manual workaround)

First Play after a cold cabinet boot triggers
`RG grip didn't initialize` and the cabinet shuts down. This is
**repeatable**. Workaround:

1. Cold-boot the cabinet.
2. **Restart it immediately** (without pressing Play).
3. Now Play works.

Alternative: open the OnRobot URCap UI from the pendant, click
"Connect" / re-init the RG grip before pressing Play.

## End-to-end first-time bring-up (mirrored mode)

Run these in order. Each step has a verification you should check
before moving on.

```powershell
# --- Windows host ---
# 1. Mirrored mode + firewall override (one-time)
notepad $env:USERPROFILE\.wslconfig
# paste the Level 1 .wslconfig contents, save
wsl --shutdown

# 2. Firewall + Hyper-V override (PowerShell as Admin, one-time)
New-NetFirewallRule -DisplayName "UR External Control 50001-50002" `
    -Direction Inbound -Action Allow -Protocol TCP -LocalPort 50001-50002
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
    -DefaultInboundAction Allow

# 3. Basic reachability (every session)
ping 192.168.1.100
Test-NetConnection 192.168.1.100 -Port 30004   # TcpTestSucceeded: True
```

```bash
# --- WSL Ubuntu-22.04 ---
# 4. Reachability from inside WSL
ping -c 3 192.168.1.100
nc -vz 192.168.1.100 30004     # succeeded -> good
nc -vz 192.168.1.100 29999     # succeeded -> good

# 5. Extract the cell's factory calibration (one-time per cell)
source /opt/ros/humble/setup.bash
source ~/ur_rg6_ws/install/setup.bash
ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=192.168.1.100 \
    target_filename:="${HOME}/ur_rg6_ws/src/ur10e_rg6_moveit_config/config/ur10e_cell_calibration.yaml"
# (rebuild moveit_config so the new yaml ends up in install/)
colcon build --packages-select ur10e_rg6_moveit_config --symlink-install
source install/setup.bash

# 6. Launch the full stack against real hardware
ros2 launch ur10e_rg6_moveit_config full_stack.launch.py \
    use_fake_hardware:=false \
    robot_ip:=192.168.1.100
```

7. On the pendant: open `external_control.urp`, press **Play**.
8. In the WSL terminal you should see:
   ```
   [ur_robot_driver]: Robot connected to reverse interface
   [ur_robot_driver]: Ready to receive control commands
   ```
9. From a separate WSL terminal, run the minimal smoke test
   (dry-run first to see exactly what would be commanded):
   ```bash
   python3 ~/ur_rg6_ws/tests/real_hw_smoke.py             # dry-run
   python3 ~/ur_rg6_ws/tests/real_hw_smoke.py --yes       # real arm + sim gripper (NO URCap call)
   ```
   This does HOME → +3 cm TCP-Z → close → HOME → -3 cm → open → HOME
   at 5 % speed. Joint perturbation is hard-capped at 0.10 rad
   (≈ 6 cm) regardless of CLI args. **Use this BEFORE the gripper
   URScript path — it isolates the arm bring-up from the URCap.**
10. Add the URCap gripper path:
    ```bash
    python3 ~/ur_rg6_ws/tests/real_hw_smoke.py --yes --real-gripper --force 25
    ```
11. Gripper-only sweep (no arm motion):
    ```bash
    python3 ~/ur_rg6_ws/tests/gripper_test.py --no-arm --real --force 25 --widths 100 80 100
    ```
12. One pick-place cycle at low force:
    ```bash
    python3 ~/ur_rg6_ws/tests/play_pickplace.py --real-gripper --force 25 --max 1
    ```
13. Full 10-cycle program at normal force:
    ```bash
    python3 ~/ur_rg6_ws/tests/play_pickplace.py --real-gripper --force 40
    ```

## Diagnostic recipes

### Reverse channel won't connect ("Connection refused on 50001/50002")

```bash
# From WSL — is the driver actually listening?
ss -ltn '( sport = :50001 or sport = :50002 )'
# If empty, the driver didn't start its server. Check launch logs.

# From Windows PowerShell — can the cabinet reach the listen IP?
Test-NetConnection 192.168.1.35 -Port 50002
# If False: firewall (Level 1) or netsh port-proxy missing (Level 3).
```

If WSL shows the listener but the cabinet still fails:
1. Check the URCap "Host IP" field matches your networking level.
2. Wireshark on the Windows interface, filter `tcp.port == 50002` —
   does the SYN arrive? If yes but no SYN-ACK, firewall. If no SYN at
   all, the URCap is dialing the wrong IP.

### Driver connects then drops repeatedly

Almost always the OnRobot URCap cold-boot quirk. Stop the program,
restart the cabinet, try again.

If the cold-boot fix doesn't help, check:
- `ros2 control list_controllers` — `scaled_joint_trajectory_controller`
  should be `active`. If `inactive`, switch it.
- Pendant in **Remote Control** mode (top-right toggle).
- No other ROS 2 driver instance running (`pgrep -f ros2_control_node`).

### Cabinet TCP all timeout (services not enabled)

You'll see this on a fresh / factory-reset cabinet. Pendant
`Settings → Security → enable Services`. No restart needed.

### Multicast / DDS issues (multi-machine ROS 2)

If a remote node can't see the WSL driver:
- Set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on BOTH ends, OR
- Use FastDDS with a discovery server. Easiest workaround for WSL2
  mirrored mode: run a `fastdds discovery -i 0` on a Linux host on
  the LAN, point both ends at it via `FASTRTPS_DEFAULT_PROFILES_FILE`.
- Or escalate to Level 2 (bridged) where multicast works natively.

### Why mirrored mode "works" for UR but not for ROS 2 inter-machine

UR uses unicast TCP — mirrored mode shares the host's TCP stack so
inbound connections to the listen-on-WSL ports are forwarded by the
host. DDS uses UDP multicast for discovery, and mirrored mode loses
multicast receives. Different transport, different gotcha.

## What I (Claude) cannot do — user-only tasks

These need a human at the cell / Windows admin shell:

| # | Task | Why I can't |
|---|---|---|
| 1 | Pendant — enable services in Settings → Security | Physical pendant access |
| 2 | Pendant — install / configure External Control URCap | Pendant UI |
| 3 | Pendant — create `external_control.urp` program | Pendant UI |
| 4 | Pendant — switch to Remote Control mode | Pendant toggle |
| 5 | Pendant — work around OnRobot URCap cold-boot bug | Power cycle |
| 6 | Pendant — press Play on the program | Physical button |
| 7 | Windows — install Hyper-V feature (Level 2) | Admin + reboot |
| 8 | Windows — create the external VMSwitch (Level 2) | Admin PowerShell |
| 9 | Windows — add the inbound firewall rule | Admin PowerShell |
| 10 | Windows — `Set-NetFirewallHyperVVMSetting` override | Admin PowerShell |
| 11 | Windows — set up `netsh portproxy` (Level 3) | Admin PowerShell |
| 12 | Cell — plug the laptop Ethernet into the cabinet | Physical |
| 13 | Cell — verify ICMP + TCP probe from PowerShell | Physical proximity to confirm |
| 14 | Verify "Robot connected to reverse interface" appears in driver logs after pressing Play | Need someone at the pendant to press Play |

I can prepare the launch commands, build the workspace, write configs,
generate calibration extraction commands — but the bring-up actions
above need you.

## Sources

- [espenakk/ros2-wsl2-guide](https://github.com/espenakk/ros2-wsl2-guide) — Hyper-V external switch + bridged WSL2 setup
- [ROS Answers Q413680 — ROS2 + WSL2 external communication](https://answers.ros.org/question/413680/) — the canonical "WSL multicast is broken" thread; recommends the YouTube tutorial
- [YouTube NOOUfsExYCE](https://www.youtube.com/watch?v=NOOUfsExYCE) — direct hardware NIC access for WSL2 (video, no transcript available to fetch)
- [microsoft/WSL #10614 — UDP multicast not working in mirrored mode](https://github.com/microsoft/WSL/discussions/10614) — confirms mirrored multicast loss; gives the `Set-NetFirewallHyperVVMSetting` override
- [microsoft/WSL #9227 — networkingMode=bridged fails to start](https://github.com/microsoft/WSL/discussions/9227) — bridged needs Win 11 Pro
- [randombytes.substack.com/bridged-networking-under-wsl](https://randombytes.substack.com/p/bridged-networking-under-wsl) — bridged setup walkthrough
- [Universal_Robots_ExternalControl_URCap #24 — reverse-socket WSL2 issue](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/issues/24) — same failure mode (WSL2 IP doesn't match what cabinet dials)
- [UR ROS 2 Driver — startup docs](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/startup.html) — official launch reference
- [UR calibration — ur_calibration package](https://docs.ros.org/en/humble/p/ur_calibration/) — calibration extraction
- `D:\robot_ws\robots\outputs\2026-05-09\SESSION_CLOSE.md` — verified cell config for THIS cell (IPs, MAC, Polyscope version, cabinet TCP-filter gotcha)
- `D:\robot_ws\robots\wiki\ur10e_rg6\path_b_deploy.md` — fallback URScript deploy workflow (when ROS 2 streaming is wrong fit)
