#!/usr/bin/env python3
import os
import subprocess
import sys

# Match this exactly to your webcam's ID_SERIAL
TARGET_SERIAL = "046d_HD_Pro_Webcam_C920"
IMAGE_WIDTH = "640"
IMAGE_HEIGHT = "480"

def find_logitech_video_device():
    base_path = '/sys/class/video4linux'
    for dev in sorted(os.listdir(base_path)):
        video_path = f"/dev/{dev}"
        try:
            udev_info = subprocess.check_output(
                ['udevadm', 'info', '--query=all', '--name', video_path],
                text=True
            )
            if f"ID_SERIAL={TARGET_SERIAL}" in udev_info:
                return video_path
        except subprocess.CalledProcessError:
            continue
    return None

def main():
    video_device = find_logitech_video_device()
    if not video_device:
        print(f"❌ Logitech webcam with serial '{TARGET_SERIAL}' not found.")
        sys.exit(1)

    print(f"✅ Found Logitech webcam at {video_device}")
    cmd = [
        "rosrun", "usb_cam", "usb_cam_node",
        f"_video_device:={video_device}",
        f"_image_width:={IMAGE_WIDTH}",
        f"_image_height:={IMAGE_HEIGHT}",
        f"_pixel_format:=yuyv",
        f"_camera_frame_id:=usb_cam_1",
        f"_focus_auto:=0",
        f"_focus_absolute:=30",
        f"usb_cam/image_raw:=wrist_camera/image_raw"
    ]

    print("🚀 Running:", " ".join(cmd))
    subprocess.run(cmd)

if __name__ == "__main__":
    main()


    # "Webcam": f"rosrun usb_cam usb_cam_node \
    #             _video_device:=/dev/video0 \
    #             _image_width:=640 \
    #             _image_height:=480 \
    #             _pi/hooller": f"roslaunch inspire_hand hand_control.launch",