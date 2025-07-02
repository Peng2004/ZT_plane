ROS node for the Raspberry Pi Camera Module. Works with both the V1.x and V2.x versions of the module. We recommend using the v2.x cameras as they have better auto gain, and the general image quality is better.
`raspicam_node` 是一个用于通过 Raspberry Pi 摄像头进行图像采集和传输的项目，主要应用于无人机（UAV）或机器人系统中。该项目基于 ROS（Robot Operating System）开发，支持通过 GStreamer 和 RTP 协议进行视频流的传输，并提供了一些基本的服务接口用于飞行控制、数据发布等功能。

## Installation
## 主要功能

A binary can be found at https://packages.ubiquityrobotics.com/ follow the instructions there to add the repository.
- **摄像头支持**：使用 Raspberry Pi 摄像头进行图像采集。
- **视频流传输**：通过 GStreamer 和 RTP 协议将视频流传输到指定的端口。
- **飞行控制服务**：提供起飞、降落、飞行控制等服务接口。
- **数据发布**：支持将摄像头采集的数据发布到 ROS 系统中供其他节点使用。
- **多线程处理**：使用多线程机制提高图像处理和数据传输的效率。

Then run `sudo apt install ros-kinetic-raspicam-node`
## 安装

## Build Intructions
If you want to build from source instead of using the binary follow this section.
### 依赖项

This node is primarily supported on ROS Kinetic, and Ubuntu 16.04, and that is what these instuctions presume.
- ROS (Robot Operating System)
- GStreamer
- OpenCV
- RTP 库

Go to your catkin_ws `cd ~/catkin_ws/src`.
### 安装步骤

Download the source for this node by running
1. 安装 ROS（如果尚未安装）：
   ```bash
   sudo apt-get install ros-<your_ros_distro>-desktop-full
   ```

`git clone https://github.com/UbiquityRobotics/raspicam_node.git`
2. 安装 GStreamer：
   ```bash
   sudo apt-get install libgstreamer1.0-dev
   ```

There are some dependencies that are not recognized by ros, so you need to create the file `/etc/ros/rosdep/sources.list.d/30-ubiquity.list` and add this to it.
```
yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml
```

Then run `rosdep update`.

Install the ros dependencies,

```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Compile the code with `catkin_make`.

## Running the Node
Once you have the node built, you can run it using a launch file.
3. 安装 OpenCV：
   ```bash
   sudo apt-get install libopencv-dev
   ```

For a V2.x camera, run `roslaunch raspicam_node camerav2_1280x960.launch`
4. 安装 RTP 库：
   ```bash
   sudo apt-get install libortp-dev
   ```

For a V1.x camera, run `roslaunch raspicam_node camerav1_1280x720.launch`
5. 将 `raspicam_node` 添加到您的 ROS 工作空间并编译：
   ```bash
   cd ~/catkin_ws/src
   git clone https://gitee.com/tt/raspicam_node.git
   cd ..
   catkin_make
   ```

Use `rqt_image_view` on a connected computer to view the published image.
## 使用方法

## Configuring the node with dynamic reconfigure
The `raspicam_node` supports dynamically reconfiguring the camera parameters.
### 启动摄像头节点

Run the dynamic reconfigure node on a connected computer:

```
rosrun rqt_reconfigure rqt_reconfigure
```bash
roslaunch raspicam_node uavdata.launch
```

It should bring up a user interface like the one below.  Paramaters can be dynamically adjusted via this interface.

![rqt_reconfigure](reconfigure_raspicam_node.png)


## Troubleshooting
1. Make sure that your user is in the `video` group by running `groups|grep video`.

2. If you get an error saying: `Failed to create camera component`,
make sure that the camera cable is properly seated on both ends, and that the cable is not missing any pins.

3. If the publish rate of the image over the network is lower than expected, consider using a lower resolution to reduce the amount of bandwidth required.

## Node Information

Topics:

* `~/image/compressed`:
  Publishes `sensor_msgs/CompressedImage` with jpeg from the camera module.

* `~/image`:
  Publishes `sensor_msgs/Image` from the camera module (if parameter `enable_raw` is set).

* `~/motion_vectors`:
  Publishes `raspicam_node/MotionVectors` from the camera module (if parameter `enable_imv` is set).

* `~/camera_info`:
  Publishes `sensor_msgs/CameraInfo` camera info for each frame.

Services:
### 视频流传输

* `~/set_camera_info`: Used to update calibration info for the camera.
视频流可以通过 GStreamer 进行传输，具体配置可以在 `pipline.py` 文件中找到。以下是一个示例函数：

Parameters:

* `~private_topics` (bool): By default the topics are private, meaning the node name will be added in front of every topic name.
If you don't want the topics to be private, you can set this parameter to "true".
This parameter is mainly present in order to keep backward compatibility.

* `~camera_frame_id` (tf frame): The frame identifier to associate the camera.

* `~camera_info_url`: The URL of the camera calibration `.yaml` file.

* `~camera_name` (string): The name of the camera, should match with name in camera_info file.

* `~framerate` (fps): Framerate to capture at. Maximum 90fps

* `~height` (pixels): Height to capture images at.

* `~width` (pixels): Width to capture images at.

* `~quality` (0-100): Quality of the captured images.

* `~enable_raw` (bool): Publish a raw image (takes more CPU and memory)

* `~enable_imv` (bool): Publish inline motion vectors computed by the GPU

* `~camera_id` (int): The camera id (only supported on Compute Module)

## Calibration
```python
def gstreamer_pipeline_v4l2(
    udp_port=5001,
    display_width=1280,
    display_height=800,
    framerate=15,
    flip_method=0,
):
    return (
        "v4l2src ! "
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "videoconvert ! "
        "x264enc bitrate=2048 ! "
        "rtph264pay config-interval=1 pt=96 ! "
        "udpsink host=127.0.0.1 port=%d" %
        (display_width, display_height, framerate, udp_port)
    )
```

The raspicam_node package contains a calibration file for the raspberry
PI camera versions 1 and 2.
### 飞行控制服务

A tutorial
  [Monocular Camera Calibration tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
shows how to calibrate a single camera.
项目中提供了飞行控制的服务接口，例如起飞、降落、速度控制等。以下是一个简单的服务调用示例：

The
  [8x6 checkerboard](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf)
and the
  [7x6 checkerboard](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check_7x6_108mm.pdf)
are rather large and require specialized printers to print out at
full scale.  They can be printed on more common printer sizes
with auto scaling turned on.  Be sure to carefully measure the
square size in millimeters and convert to meters by dividing by 1000.
```cpp
bool serverCB(ttauav_node::flight::Request &req, ttauav_node::flight::Response &res) {
    // 处理飞行控制请求
    return true;
}
```

Running calibration requires raw publishing enabled. Add `enable_raw:=true` to the camera roslaunch command.
### 数据发布

If you are not sure which launch file to use `camerav2_1280x960_10fps.launch` is probably what you are looking for.
通过 `uavData_publish` 类可以将摄像头采集的数据发布到 ROS 系统中：

On the Pi
```
roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true
```cpp
int main(int argc, char **argv) {
    ros::init(argc, argv, "raspicam_node");
    ros::NodeHandle nh;
    uavData uav_data;
    uavData_publish publisher(&nh, &uav_data);
    publisher.proxy();
    ros::spin();
    return 0;
}
```

On your workstation:
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.074 image:=/raspicam_node/image camera:=/raspicam_node
```
## 贡献

## Motion vectors
欢迎贡献代码！请遵循以下步骤：

The raspicam_node is able to output [motion vectors](https://www.raspberrypi.org/blog/vectors-from-coarse-motion-estimation/) calculated by the Raspberry Pi's hardware video encoder. These motion vectors can be used for various applications such as motion detection.
1. Fork 本项目。
2. 创建一个新的分支。
3. 提交您的更改。
4. 创建一个 Pull Request。

On the Pi, add `enable_imv:=true` to the camera roslaunch command:
## 许可证

```
roslaunch raspicam_node camerav2_410x308_30fps.launch enable_imv:=true
```
本项目使用 [MIT License](LICENSE)，请查看 LICENSE 文件以获取更多信息。

On your workstation, build raspicam_node so that the `MotionVectors` ROS message is recognized by Python:
## 作者

```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
- **tt**

Finally, run script `imv_view.py` to visualize the motion vectors:
## 致谢

```
rosrun raspicam_node imv_view.py
```
感谢 Raspberry Pi 和 ROS 社区提供的资源和支持。
\ No newline at end of file