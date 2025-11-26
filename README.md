# gps_trust
ROS2 nodes for checking the trust level of GPS messages

# GPS Trust – Automated ROS 2 GNSS Stack Setup

This repository provides a full automation pipeline for setting up a GPS-Trust station on Ubuntu 24.04 or Raspberry Pi 5.

It configures:
- a dedicated `gpstrust` system user  
- ROS 2 workspaces for both **gps_trust** and **ublox_dgnss**  
- environment configuration (`/etc/gpstrust.env`)  
- a managed `systemd` service (`gpstrust.service`) that launches and supervises the stack

---

## 🧭 Overview

The system runs three ROS 2 launch files:
1. **gps_trust_satellite** – hardware driver for u-blox F9P or X20P  
2. **ntrip_client** – NTRIP correction stream handler  
3. **gps_trust_main** – main trust computation and publishing node  

Each component runs under the `gpstrust` service account and writes to `/var/log/gpstrust`.

---

## 🚀 Quick Install

Follow ROS2 install docs
[ROS2 Installation Guide](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html)

```bash
sudo apt install git python3-colcon-common-extensions python3-rosdep ros2-rolling-ros-base
git clone https://github.com/aussierobots/gps_trust.git && cd gps_trust
sudo -E ./setup/install_gpstrust_service.sh
```

During setup you will be prompted for:
- ROS install prefix (default /opt/ros/rolling)
- Device type (F9P or X20P)
- NTRIP credentials and mountpoint
- API key for GPS-Trust device telemetry

The script:
- creates the gpstrust user
- clones gps_trust and ublox_dgnss into /home/gpstrust
- builds both workspaces with colcon build
- writes /etc/gpstrust.env with all required variables
- installs and enables the gpstrust.service

Note:

If you the environment variables already set, it should default to them
