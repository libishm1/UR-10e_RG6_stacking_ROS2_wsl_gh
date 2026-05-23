#!/bin/bash
# 🧹 Kill any leftover processes
echo "🔪 Cleaning old processes..."
taskkill.exe /IM vcxsrv.exe /F >/dev/null 2>&1
pkill -9 rviz2 >/dev/null 2>&1
pkill -9 ogre >/dev/null 2>&1

# 🪟 Start a fresh X server (Windows side)
echo "🚀 Starting X server..."
powershell.exe -Command "Start-Process 'C:\\Program Files\\VcXsrv\\vcxsrv.exe' ':0 -multiwindow -ac -clipboard'"

# ⏱ Wait a few seconds for X to start
sleep 3

# 🌐 Export DISPLAY & GL variables
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
export LIBGL_ALWAYS_INDIRECT=1
export QT_X11_NO_MITSHM=1

echo "✅ DISPLAY set to $DISPLAY"
echo "🧱 Launching UR10e + RG6 RViz visualization..."

# 📦 Launch URDF in RViz
source /opt/ros/humble/setup.bash
source ~/ur_rg6_ws/install/setup.bash
ros2 launch ur_description view_ur10e_rg6_full.launch.py
