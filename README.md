# ROS2tcp
[FPGA-Based-EdgeAI-Prototypes](https://github.com/DYGV/FPGA-Based-EdgeAI-Prototypes/)のTCPサーバとの通信をROS 2ノード内で行うパッケージ  

https://github.com/DYGV/ros2tcp-edgeAI/assets/8480644/545b7924-9846-4fd9-8228-a07ef007d903

![rqt](./docs/rosgraph.png)



## 起動手順
### image publisher  
```
ros2 run video_sender video_publisher --ros-args -p video_file:=/dev/video0 -p hz:=30
```  
## face\_detection\_tcp
```
ros2 run face_detection_tcp face_detection_tcp --ros-args -p server_addr:=192.168.0.2 -p server_port:=54321 -p show_gui:=true
```  
## pose\_estimation\_tcp
```
ros2 run pose_estimation_tcp pose_estimation_tcp --ros-args -p server_addr:=192.168.0.163 -p server_port:=54321 -p show_gui:=true
```  
