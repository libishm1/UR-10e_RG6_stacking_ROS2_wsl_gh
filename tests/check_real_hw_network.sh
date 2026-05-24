#!/usr/bin/env bash
# Pre-flight network check for real-hardware UR10e bring-up from WSL2.
# Runs through the Level 0/1 checks from docs/WSL2_UR10e_NETWORKING.md.
#
# Usage:
#   ./check_real_hw_network.sh [robot_ip]
# Default robot_ip is 192.168.1.100 (per D:\robot_ws SESSION_CLOSE.md).

ROBOT_IP="${1:-192.168.1.100}"

# Make the script work whether or not it's sourced
RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; NC=$'\033[0m'

PASS=0
FAIL=0

check() {
    local name="$1"; shift
    if "$@" >/dev/null 2>&1; then
        printf "  ${GREEN}PASS${NC}  %s\n" "$name"
        PASS=$((PASS+1))
    else
        printf "  ${RED}FAIL${NC}  %s\n" "$name"
        FAIL=$((FAIL+1))
    fi
}

warn() {
    local name="$1"; shift
    if "$@" >/dev/null 2>&1; then
        printf "  ${GREEN}PASS${NC}  %s\n" "$name"
        PASS=$((PASS+1))
    else
        printf "  ${YELLOW}WARN${NC}  %s\n" "$name"
    fi
}

echo "=== Real-hardware network pre-flight for ${ROBOT_IP} ==="
echo
echo "[1/4] WSL networking mode"
grep -E '^networkingMode' /mnt/c/Users/libish\ m/.wslconfig 2>/dev/null \
    || printf "  ${YELLOW}WARN${NC}  Could not read .wslconfig from /mnt/c/Users/libish m/\n"
echo "  WSL eth0 IP(s): $(ip -4 -o addr show eth0 2>/dev/null | awk '{print $4}' | tr '\n' ' ')"
echo

echo "[2/4] Reachability to cabinet ${ROBOT_IP}"
check "ICMP (ping)"       ping -c 2 -W 2 "${ROBOT_IP}"
check "TCP 29999 (dashboard)" nc -z -w 3 "${ROBOT_IP}" 29999
check "TCP 30001 (primary)"   nc -z -w 3 "${ROBOT_IP}" 30001
check "TCP 30002 (secondary)" nc -z -w 3 "${ROBOT_IP}" 30002
check "TCP 30004 (RTDE)"      nc -z -w 3 "${ROBOT_IP}" 30004
warn  "TCP 22 (SSH)"          nc -z -w 3 "${ROBOT_IP}" 22
echo

echo "[3/4] Dashboard handshake (port 29999)"
DB_REPLY=$(printf 'PolyscopeVersion\nrobotmode\nis in remote control\nsafetystatus\nquit\n' \
    | nc -w 3 "${ROBOT_IP}" 29999 2>/dev/null)
if [ -n "$DB_REPLY" ]; then
    echo "$DB_REPLY" | sed 's/^/    /'
    if echo "$DB_REPLY" | grep -qi 'remote control: true'; then
        printf "  ${GREEN}PASS${NC}  cabinet is in Remote Control mode\n"
        PASS=$((PASS+1))
    else
        printf "  ${YELLOW}WARN${NC}  cabinet NOT in Remote Control mode (pendant top-right toggle)\n"
    fi
    if echo "$DB_REPLY" | grep -qi 'safetystatus: NORMAL'; then
        printf "  ${GREEN}PASS${NC}  safety is NORMAL\n"
        PASS=$((PASS+1))
    else
        printf "  ${RED}FAIL${NC}  safety NOT normal — clear protective stop / E-stop\n"
        FAIL=$((FAIL+1))
    fi
else
    printf "  ${RED}FAIL${NC}  no Dashboard reply — services likely disabled on pendant\n"
    FAIL=$((FAIL+1))
fi
echo

echo "[4/4] Reverse-channel listener readiness (run AFTER ros2 launch)"
if ss -ltn 2>/dev/null | grep -qE ':50001\b|:50002\b'; then
    printf "  ${GREEN}PASS${NC}  driver is listening on 50001/50002\n"
    PASS=$((PASS+1))
else
    printf "  ${YELLOW}WARN${NC}  no listener on 50001/50002 (expected if driver hasn't launched yet)\n"
fi
echo

echo "=== Summary: ${PASS} pass, ${FAIL} fail ==="
if [ "${FAIL}" -gt 0 ]; then
    echo
    echo "Failures need fixing before launching ur_robot_driver. See"
    echo "docs/WSL2_UR10e_NETWORKING.md → \"Diagnostic recipes\"."
    exit 1
fi
