# raspicam_node

`raspicam_node` 是一个用于通过 Raspberry Pi 摄像头进行图像采集和传输的项目，主要应用于无人机（UAV）或机器人系统中。该项目基于 ROS（Robot Operating System）开发，支持通过 GStreamer 和 RTP 协议进行视频流的传输，并提供了一些基本的服务接口用于飞行控制、数据发布等功能。

## 主要功能

- **摄像头支持**：使用 Raspberry Pi 摄像头进行图像采集。
- **视频流传输**：通过 GStreamer 和 RTP 协议将视频流传输到指定的端口。
- **飞行控制服务**：提供起飞、降落、飞行控制等服务接口。
- **数据发布**：支持将摄像头采集的数据发布到 ROS 系统中供其他节点使用。
- **多线程处理**：使用多线程机制提高图像处理和数据传输的效率。

## 安装

### 依赖项

- ROS (Robot Operating System)
- GStreamer
- OpenCV
- RTP 库

### 安装步骤

1. 安装 ROS（如果尚未安装）：
   ```bash
   sudo apt-get install ros-<your_ros_distro>-desktop-full
   ```

2. 安装 GStreamer：
   ```bash
   sudo apt-get install libgstreamer1.0-dev
   ```

3. 安装 OpenCV：
   ```bash
   sudo apt-get install libopencv-dev
   ```

4. 安装 RTP 库：
   ```bash
   sudo apt-get install libortp-dev
   ```

5. 将 `raspicam_node` 添加到您的 ROS 工作空间并编译：
   ```bash
   cd ~/catkin_ws/src
   git clone https://gitee.com/tt/raspicam_node.git
   cd ..
   catkin_make
   ```

## 使用方法

### 启动摄像头节点

```bash
roslaunch raspicam_node uavdata.launch
```

### 视频流传输

视频流可以通过 GStreamer 进行传输，具体配置可以在 `pipline.py` 文件中找到。以下是一个示例函数：

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

### 飞行控制服务

项目中提供了飞行控制的服务接口，例如起飞、降落、速度控制等。以下是一个简单的服务调用示例：

```cpp
bool serverCB(ttauav_node::flight::Request &req, ttauav_node::flight::Response &res) {
    // 处理飞行控制请求
    return true;
}
```

### 数据发布

通过 `uavData_publish` 类可以将摄像头采集的数据发布到 ROS 系统中：

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

## 贡献

欢迎贡献代码！请遵循以下步骤：

1. Fork 本项目。
2. 创建一个新的分支。
3. 提交您的更改。
4. 创建一个 Pull Request。

## 许可证

本项目使用 [MIT License](LICENSE)，请查看 LICENSE 文件以获取更多信息。

## 致谢

感谢 Raspberry Pi 和 ROS 社区提供的资源和支持。
