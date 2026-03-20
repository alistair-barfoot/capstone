# G1 Camera Feed Startup Procedure for Unity

This guide explains how to start the Unitree G1 camera pipeline so Unity can receive the camera feed.

## Working Setup

The working setup uses:

- **Terminal 1**: ROS TCP Endpoint
- **Terminal 2**: RealSense camera node
- **Terminal 3**: JPEG compressor node for the color image

Unity should subscribe to:
```
/color/image_raw/compressed
```

with message type:
```
sensor_msgs/msg/CompressedImage
```

## Overview

The final pipeline is:

```
Unitree G1 RealSense camera
    -> /color/image_raw
    -> image_compressor_no_bridge.py
    -> /color/image_raw/compressed
    -> ros_tcp_endpoint
    -> Unity
```

## Robot IP

The robot was reachable at:
```
192.168.123.164
```

The failed IPs were:
- `192.168.1.133`
- `192.1.1.133`
- `localhost`

So use:
```bash
ssh unitree@192.168.123.164
```

## Important Notes Before Starting

Choose ROS 2 Foxy when prompted:
```
ros:foxy(1) noetic(2) ?
1
```

In every ROS terminal, run:
```bash
cd ~/capstone_ws
source install/setup.bash
```

**Important requirements:**
- You need all required terminals running at the same time
- If Unity is using the compressed topic, the compressor node must be running
- The camera node can fail if the device is already in use. If that happens, stop other camera processes and retry

## Terminal 1: Start ROS TCP Endpoint

SSH into the robot:
```bash
ssh unitree@192.168.123.164
```

Then:
```bash
cd ~/capstone_ws
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.1.133 -p ROS_TCP_PORT:=10000
```

### What this does

This starts the Unity TCP bridge on port 10000.

### Expected behavior

You should see lines like:
```
[INFO] [UnityEndpoint]: Starting server on 192.168.1.133:10000
[INFO] [UnityEndpoint]: Connection from 192.168.1.229
```

If Unity is configured correctly, you should also see subscriber/publisher registration messages.

### Important

At first, Unity was incorrectly registering:
```
/color/image_raw/compressed as sensor_msgs.msg.Image
```

**That is wrong.**

The correct working registration was:
```
/color/image_raw/compressed as sensor_msgs.msg.CompressedImage
```

So in Unity, make sure the topic is:
```
/color/image_raw/compressed
```

and the message type is:
```
sensor_msgs/msg/CompressedImage
```

## Terminal 2: Start the RealSense camera

SSH into the robot in a second terminal:
```bash
ssh unitree@192.168.123.164
```

Then run:
```bash
cd ~/capstone_ws
source install/setup.bash
ros2 run realsense2_camera realsense2_camera_node align_depth.enable:=true
```

### Expected behavior

When it works, you should eventually see:
```
[INFO] [RealSenseCameraNode]: RealSense Node Is Up!
```

You should also see topics such as:
- `/color/image_raw`
- `/depth/image_rect_raw`
- `/color/camera_info`

### Common failure

The camera sometimes failed with:
```
xioctl(VIDIOC_S_FMT) failed, errno=16 Last Error: Device or resource busy
```

That means something else already had the camera open.

### Fix

If that happens:
1. Stop the current attempt with `Ctrl+C`
2. Wait a few seconds
3. Retry the same command

Eventually the node came up successfully.

## Terminal 3: Start the image compressor

SSH into the robot in a third terminal:
```bash
ssh unitree@192.168.123.164
```

Then run:
```bash
cd ~/capstone_ws
source install/setup.bash
python3 image_compressor_no_bridge.py
```

### What this does

This subscribes to:
```
/color/image_raw
```

and publishes compressed JPEG frames to:
```
/color/image_raw/compressed
```

### Expected behavior

You should see logs like:
```
[INFO] [image_compressor]: Image Compressor Node Started (no cv_bridge)
[INFO] [image_compressor]:   Input:  /color/image_raw
[INFO] [image_compressor]:   Output: /color/image_raw/compressed
[INFO] [image_compressor]:   JPEG Quality: 80
```

And then repeated compression stats such as:
```
Frame 60: 900.0 KB -> 71.4 KB | 12.6x compression | 92.1% bandwidth saved
```

That means the compressor is working correctly.

## Optional: Start NoMachine

This was run from the robot home directory, not from `capstone_ws`.

SSH into the robot and run:
```bash
cd ~
./nomachine.sh
```

If you try running it from `~/capstone_ws`, it will fail with:
```
-bash: ./nomachine.sh: No such file or directory
```

So only run it from: `~`

This step is optional for the camera pipeline.

## Verifying the topics

Once the RealSense node is running, you can inspect topics with:
```bash
ros2 topic list
```

The relevant topics included:
- `/color/camera_info`
- `/color/image_raw`
- `/color/image_raw/compressed`
- `/depth/camera_info`
- `/depth/image_rect_raw`

To check the raw image bandwidth:
```bash
ros2 topic bw /color/image_raw
```

This showed about: `~27 MB/s`

Which is exactly why compression was needed.

## Unity setup

Unity should be configured to subscribe to:
```
/color/image_raw/compressed
```

with type:
```
sensor_msgs/msg/CompressedImage
```

**Do not subscribe to `/color/image_raw/compressed` as `sensor_msgs/Image`**. That caused disconnects and endpoint errors.

### Correct setup summary

- **Topic**: `/color/image_raw/compressed`
- **Message type**: `CompressedImage`
- **ROS TCP Endpoint port**: `10000`

## Startup order

Use this order every time:

### 1. Terminal 1

Start ROS TCP endpoint:
```bash
ssh unitree@192.168.123.164
cd ~/capstone_ws
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.1.133 -p ROS_TCP_PORT:=10000
```
### 2. Terminal 2

Start RealSense node:
```bash
ssh unitree@192.168.123.164
cd ~/capstone_ws
source install/setup.bash
ros2 run realsense2_camera realsense2_camera_node
```

Wait until you see:
```
RealSense Node Is Up!
```
### 3. Terminal 3

Start compressor:
```bash
ssh unitree@192.168.123.164
cd ~/capstone_ws
source install/setup.bash
python3 image_compressor_no_bridge.py
```
### 4. Unity

Run the Unity project and connect to the endpoint.

## Troubleshooting

### Problem: SSH fails

Use the correct robot IP:
```
192.168.123.164
```

### Problem: RealSense says "device or resource busy"

Another process is already using the camera.

Fix:

1. Stop other camera processes
2. Retry `realsense2_camera_node`

### Problem: `/color/image_raw/compressed` exists but no data comes through

Check that `image_compressor_no_bridge.py` is running.

### Problem: Unity connects, then disconnects

Most likely cause: wrong message type in Unity.

**Wrong:**
```
/color/image_raw/compressed -> Image
```

**Correct:**
```
/color/image_raw/compressed -> CompressedImage
```

### Problem: Raw feed is too slow

That is expected. Raw color was around: `~27 MB/s`

Use the compressed topic instead.

### Problem: ROS TCP Endpoint throws topic list errors

You saw errors like:

```
AttributeError: 'NoneType' object has no attribute 'msg'
```

That showed up during some bad connection attempts, but the main practical fix was still the same: make Unity use the correct topic type and reconnect cleanly.

## Final working state

You know the procedure is working when:

- **Terminal 1** shows Unity connected
- **Terminal 2** shows `RealSense Node Is Up!`
- **Terminal 3** shows repeated frame compression logs
- **Unity** subscribes to `/color/image_raw/compressed` as `CompressedImage`

## Minimal Quick-start Version

### Terminal 1
```bash
ssh unitree@192.168.123.164
cd ~/capstone_ws
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.1.133 -p ROS_TCP_PORT:=10000
```

### Terminal 2
```bash
ssh unitree@192.168.123.164
cd ~/capstone_ws
source install/setup.bash
ros2 run realsense2_camera realsense2_camera_node
```

### Terminal 3
```bash
ssh unitree@192.168.123.164
cd ~/capstone_ws
source install/setup.bash
python3 image_compressor_no_bridge.py
```

### Unity Configuration

- **Topic**: `/color/image_raw/compressed`
- **Type**: `CompressedImage`

>This readme will detail both of the C# files and how they are organized

>It will also give instructions on how to set up the unity environment for this project
