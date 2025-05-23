# Inspire Hand Teleoperation

This guide will help you set up and run teleoperation for the Inspire Hand robot using Meta Quest 3.

---

## Prerequisites

### Get a machine with Ubuntu 20.04 and install ROS Noetic
If you encounter a **GPG key error** during ROS installation, run:
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
```

---

## 1. Install Inspire-Hand-Teleoperation

### Clone this repository
```bash
mkdir ~/roboskill/Hands && cd ~/roboskill/Hands
git clone https://github.com/kopiotrek/Inspire-Hand-Teleoperation.git
```

### Add Environment Variables to `.bashrc`
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export PYTHONPATH=$HOME/roboskill/Hands/Inspire-Hand-Teleoperation:\$PYTHONPATH" >> ~/.bashrc
# echo "export PYTHONPATH=$HOME/ros_ws/ik_teleop:\$PYTHONPATH" >> ~/.bashrc
echo "export PYTHONPATH=\$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages" >> ~/.bashrc
source ~/.bashrc
```

### Set Up Virtual Environment
```bash
sudo apt install python3.8-venv python3-pip
cd ~/roboskill/Hands/Inspire-Hand-Teleoperation/ik_teleop/
python3 -m venv venv_teleop
source venv_teleop/bin/activate
pip install -r requirements_teleop.txt
```

## 2. Setup Instructions for Inspire Hand Controller

### Install Dependencies & Clone Repository
```bash
cd ~/roboskill/Hands
git clone https://github.com/NYU-robot-learning/Inspire-Hand-Controller-DIME.git
sudo apt-get install cmake gcc g++ libpopt-dev ros-noetic-libpcan
```

### Build the Controller
```bash
cd ~/roboskill/Hands/Inspire-Hand-Controller-DIME
catkin_make
```

---

## 3. Install and Configure PEAK-CAN Drivers

### Install PEAK System CAN Drivers
```bash
mkdir ~/roboskill/Hands/drivers && cd ~/roboskill/Hands/drivers
wget https://www.peak-system.com/quick/PCAN-Linux-Driver
```
```bash
tar -xvzf PCAN-Linux-Driver
cd peak-linux-driver-8.20.0/
make clean
make NET=NO_NETDEV_SUPPORT
sudo make install
sudo modprobe pcan
```

### Install PCAN-Basic API
```bash
cd ~/roboskill/Hands/drivers
wget https://www.peak-system.com/quick/BasicLinux
```
```bash
tar -xvzf BasicLinux
cd PCAN-Basic_Linux-4.10.0.4/libpcanbasic/
make
sudo make install
```
> **Note:** Sometimes the driver installation needs to be redone after a PC restart.

### Test the Driver Installation
```bash
cat /proc/pcan
ls -l /dev/pcan*
```

### Test Connection to the Inspire Hand
```bash
cd ~/roboskill/Hands/Inspire-Hand-Controller-DIME
catkin_make
source devel/setup.bash
roslaunch inspire_hand inspire_hand.launch
```
Press **H** to home the robot.

---

## 4. Install ROS-TCP-Endpoint

```bash
mkdir -p ~/roboskill/Hands/src && cd ~/roboskill/Hands/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ..
catkin_make
```


---

## 5. Setup Instructions for Meta Quest

To use Meta Quest for teleoperation, follow these steps:

### Install SideQuest
1. Enable **Developer Mode** on your Meta Quest device.
2. Download **SideQuest Desktop App** from [SideQuestVR](https://sidequestvr.com/setup-howto):
   ```bash
   tar -xf SideQuest-0.10.42.tar.xz
   chmod +x SideQuest-0.10.42/sidequest
   ./SideQuest-0.10.42/sidequest --no-sandbox
   ```
3. In SideQuest, click **"Install APK from folder on your computer"**, then select `InspireHandTeleoperation.apk`.
4. On your Meta Quest, go to **Apps > Unknown Sources** to find and launch the installed application.
5. **Ensure your device is connected to the correct network!**

## 6. Starting the Teleoperation

1. **Ensure the Inspire Hand is connected and the driver is working.**
2. Activate the virtual environment:
   ```bash
   source ~/roboskill/Hands/Inspire-Hand-Teleoperation/ik_teleop/venv_teleop/bin/activate
   ```
3. Run the teleoperation script:
   ```bash
   python ~/roboskill/Hands/Inspire-Hand-Teleoperation/start_teleop.py --ip=<PC_IP>
   ```
4. **Press "START ALL"** in the interface.
5. Start the Inspire-Hand-Teleoperation app in Meta Quest, check the IP and otherwise just click continue everywhere, then press skip
6. **Teleoperate the hand!**

---

## Troubleshooting
- If the **PCAN driver doesn't work after a reboot**, try reinstalling it using the steps in Section 3.
- Make sure the **Inspire Hand is connected and powered on** before running teleoperation.
- Double-check that your **Meta Quest and PC are in the same network**.

---

With this guide, you should be able to successfully set up and run Inspire Hand Teleoperation. 🚀

[Example video here](https://dex-manip.github.io/videos/teleop_website.mp4)