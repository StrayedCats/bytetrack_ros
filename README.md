# bytetrack_ros

This package is a port of [ByteTrack](https://github.com/ifzhang/ByteTrack) to ROS2.
The object detector is assumed to be [YOLOX-ROS](https://github.com/Ar-Ray-code/YOLOX-ROS).


https://user-images.githubusercontent.com/53618876/145365173-29bb3ec1-b088-4ca2-9b2d-3b021a280c3b.mp4
* demo video is [MOT16-04](https://motchallenge.net/vis/MOT16-04), using yolox-s model.

## Requirements
- ROS2 Foxy+Humble
- OpenCV
- Eigen3
- [bbox_ex_msgs](https://github.com/Ar-Ray-code/bbox_ex_msgs)
- [YOLOX-ROS](https://github.com/Ar-Ray-code/YOLOX-ROS)
- OpenVINO or TensorRT
- [ros_video_player](https://github.com/fateshelled/ros_video_player)
- [detector2d](https://github.com/StrayedCats/detector2d)

## Usage (OpenVINO)
### Install
```
source /opt/ros/foxy/setup.bash
source /opt/intel/openvino_2021/bin/setupvars.sh

cd ~/ros2_ws/src
git clone https://github.com/fateshelled/bytetrack_ros
git clone https://github.com/fateshelled/ros_video_player
git clone --recursive https://github.com/Ar-Ray-code/YOLOX-ROS

# Download onnx file and Convert to IR format.
./YOLOX-ROS/weights/openvino/install.bash yolox_s
```

### RUN
```
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash

# launch with WebCam
ros2 launch bytetrack_cpp_node bytetrack_openvino.launch.py

# launch with Video File
ros2 launch bytetrack_cpp_node bytetrack_openvino.launch.py video_path:={video file path}

```
#### with detector2d(yolox_trt)
```bash 
# terminal 1
ros2 launch ros_video_player ros_video_player.launch.py video_path:="your video path" loop:=True
# terminal 2
ros2 run bytetrack_cpp_node bytetrack_cpp_node
# terminal 3
ros2 run detector2d_node detector2d_node_exec --ros-args -p load_target_plugin:=detector2d_plugins::YoloxTrt  -p yolox_trt_plugin model_path:="your weight path" -p yolox_trt_plugin.imshow_isshow:=false
# terminal 4
ros2 run bytetrack_viewer bytetrack_viewer
```


### ros graph
![rosgraph](https://user-images.githubusercontent.com/53618876/145340126-8c5fc081-9238-49f8-bace-de2854e546b7.png)
