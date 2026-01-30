# neoracer_ros2_driver

Backend ROS2 driver for neoracer v1 with support for OSRbot software stack in collaboration with Seeed Studio

---

## Table of Contents
1. [System Info](#1--system-info)
2. [Installation](#2--installation)
3. [Usage](#3--usage)
4. [Troubleshooting](#4--troubleshooting)
5. [Licensing](#5--licensing)

---

## 1 | System Info

The compute used on the Neoracer platform is the Jetson Orin Nano, which runs Jetpack 6.2. To reflash the system to an updated version of Jetpack, follow the instructions provided [here](https://wiki.seeedstudio.com/reComputer_J4012_Flash_Jetpack/)
- **OS Version**: Ubuntu 22.04.5 LTS (codename: jammy)
- **Python Version**: 3.10.12
- **ROS2 Version**: Humble

All neoracer-related systems come with the following standard naming conventions, for access convenience and compatibility with scripts. Note that the password can be changed depending on the use case.
- **Username**: racecar
- **Hostname**: neoracer
- **IP**: 192.168.1.[100 + Car ID]
- **Password**: neobotics
- **SSID**: neoracer_[Car ID]

---

## 2 | Installation

1. Install the following apt dependencies:

```sh
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-rqt-common-plugins -y
sudo apt install ros-humble-joy ros-humble-joy-linux ros-humble-teleop-twist-joy -y
sudo apt install vim tmux screen terminator -y
sudo apt install ufw -y
sudo apt install joystick -y
sudo apt install -y dkms git build-essential nvidia-l4t-kernel-headers -y
```

2. Install the following pip dependencies:

```sh
pip3 install opencv-python==4.8.1.78
pip3 install numpy==1.26.2
pip3 install nptyping==1.4.4
pip3 install jupyterlab
```

3. Set up the following aliases in `~/.bashrc`:

```sh
source ~/ros2_ws/install/setup.bash # or osracer_ws depending on the stack
alias teleop="ros2 launch neoracer_ros2_driver teleop.launch.py"
alias rqt_image_view="ros2 run rqt_image_view rqt_image_view"
alias rqt_runtime_monitor="ros2 run rqt_runtime_monitor rqt_runtime_monitor"
```

4. Set up Jupyter Lab as an autostart service

```sh
sudo ufw allow 8888 # firewall
export PATH="$HOME/.local/bin:$PATH" # fix path
sudo vim /etc/systemd/system/jupyterlab.service
```

Paste the following into the file:

```sh
[Unit]
Description=Jupyter Lab
After=network.target

[Service]
Environment=PATH=/home/nvidia/.local/bin:/usr/local/bin:/user/bin:/bin
Type=simple
User=nvidia
ExecStart=/bin/bash -c "source /home/nvidia/osracer_ws/install/setup.bash && source /opt/ros/humble/setup.bash && /home/nvidia/.local/bin/jupyter lab --no-browser --ip=0.0.0.0 --port=8888 --NotebookApp.token=''"
WorkingDirectory=/home/nvidia/jupyter_ws
Restart=always

[Install]
WantedBy=multi-user.target
```

Reload, enable, and start the service

```sh
sudo loginctl enable-linger jetson   
sudo systemctl daemon-reload
sudo systemctl enable jupyterlab.service
sudo systemctl start jupyterlab.service

sudo journalctl -u jupyterlab.service -f # to debug service
```

5. Set up joystick driver

```sh
sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4 
sudo dkms install -m xpad -v 0.4
```

6. Clone and build the ROS2 driver

```sh
cd ~/osracer_ws/src # or ~/ros2_ws depending on stack
git clone https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver.git
git clone https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver.git # lidar driver
cd ..
colcon build --symlink-install # build entire stack
source install/setup.bash
source ~/.bashrc # should see no errors here now
```

---

## 3 | Usage

1. To run the stack, use the `teleop` command in any terminal.

```sh
teleop
```

2. To access the system headless, use the Jupyter notebook that is hosted on port `8888`. The static IP for the system is `192.168.1.[100 + CAR_ID]`. A computer that is connected via the car's wifi network can therefore access the system via the web address `http://192.168.1.101:8888`.

3. Follow the instructions in `racecar-neo-installer` to install and use the frontend library to access the topics that `teleop` spawns: [https://github.com/MITRacecarNeo/racecar-neo-installer](https://github.com/MITRacecarNeo/racecar-neo-installer)

---

## 4 | Troubleshooting

If you receive an error that looks like this:

```sh
Errors were encountered while processing:
    nvidia-l4t-bootloader
    nvidia-l4t-kernel
    nvidia-l4t-kernel-headers
    ...
```

Fix by copying the folloiwng block of code into a termainal, which resets the dpkg info directory:

```sh
sudo mv /var/lib/dpkg/info/ /var/lib/dpkg/backup/
sudo mkdir /var/lib/dpkg/info/
sudo apt-get update
sudo apt-get -f install
sudo mv /var/lib/dpkg/backup/* /var/lib/dpkg/info/
sudo rm -rf /var/lib/dpkg/backup/   
sudo apt update && sudo apt upgrade -y
```

---

## 5 | Licensing

Hardware design files in this repository are licensed under **CERN-OHL-S-2.0** (CERN Open Hardware Licence Version 2 – Strongly Reciprocal).

Source Location: https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver

Warranty disclaimer: provided **as-is**, without any express or implied warranty.