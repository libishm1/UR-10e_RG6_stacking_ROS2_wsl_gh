# ur10e_rg6.ps1 — Windows launcher for the ROS 2 Humble UR10e + RG6 stack.
# Usage from PowerShell:
#   .\docker\ur10e_rg6.ps1 build
#   .\docker\ur10e_rg6.ps1 rviz
#   .\docker\ur10e_rg6.ps1 fake
#   .\docker\ur10e_rg6.ps1 real -RobotIp 192.168.1.100
#   .\docker\ur10e_rg6.ps1 rosbridge
#   .\docker\ur10e_rg6.ps1 shell
#   .\docker\ur10e_rg6.ps1 down

[CmdletBinding()]
param(
    [Parameter(Position = 0, Mandatory = $true)]
    [ValidateSet('build', 'rviz', 'fake', 'real', 'moveit', 'rosbridge', 'shell', 'down', 'rebuild')]
    [string]$Command,

    [string]$RobotIp = '192.168.1.100',
    [string]$Distro  = 'Ubuntu-22.04',
    [string]$WsPath  = '~/ur_rg6_ws'
)

$ErrorActionPreference = 'Stop'

function Invoke-Wsl {
    param([string]$BashCmd, [hashtable]$Env = @{})
    $envPrefix = ($Env.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" }) -join ' '
    if ($envPrefix) { $envPrefix = "$envPrefix " }
    $full = "cd $WsPath && $envPrefix$BashCmd"
    Write-Host "[wsl/$Distro] $full" -ForegroundColor Cyan
    wsl -d $Distro -- bash -lc $full
}

switch ($Command) {
    'build' {
        Invoke-Wsl "docker compose -f docker/docker-compose.yml --profile build_only build ur10e_rg6"
    }
    'rebuild' {
        Invoke-Wsl "docker compose -f docker/docker-compose.yml --profile build_only build --no-cache ur10e_rg6"
    }
    'rviz' {
        Invoke-Wsl "docker compose -f docker/docker-compose.yml up rviz_view"
    }
    'fake' {
        Invoke-Wsl "docker compose -f docker/docker-compose.yml up ur_fake"
    }
    'real' {
        Invoke-Wsl "docker compose -f docker/docker-compose.yml up ur_real" -Env @{ ROBOT_IP = $RobotIp }
    }
    'moveit' {
        Invoke-Wsl "docker compose -f docker/docker-compose.yml up moveit"
    }
    'rosbridge' {
        Invoke-Wsl "docker compose -f docker/docker-compose.yml up rosbridge"
    }
    'shell' {
        Invoke-Wsl "docker compose -f docker/docker-compose.yml run --rm shell"
    }
    'down' {
        Invoke-Wsl "docker compose -f docker/docker-compose.yml down"
    }
}
