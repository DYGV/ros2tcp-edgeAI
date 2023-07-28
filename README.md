# ros2tcp-edgeAI
[FPGA-Based-EdgeAI-Prototypes](https://github.com/DYGV/FPGA-Based-EdgeAI-Prototypes/)のTCPサーバとの通信をROS 2ノード内で行うパッケージ  

![demo](./docs/demo.gif)  
<p align="center">
ZynqMPで顔検出(左)、Alveo U50で姿勢推定(右)のTCPサーバを立ち上げてROS 2ノードから通信したとき
</p>


## ROS 2ノード起動手順
![rqt](./docs/rosgraph.png)  
### image\_senderパッケージ
`image`トピックに画像データをPublishするノードの立ち上げ  
```
ros2 run video_sender video_publisher --ros-args -p video_file:=/dev/video0 -p hz:=30
```  
### face\_detection\_tcpパッケージ
`image`トピックをSubscribeし、顔検出のTCPサーバへ画像送信・結果受信し、結果を`face_detection_result`トピックへPublishするノードの立ち上げ  
```
ros2 run face_detection_tcp face_detection_tcp --ros-args -p server_addr:=192.168.0.2 -p server_port:=54321 -p show_gui:=true
```  
### pose\_estimation\_tcpパッケージ
`image`トピックをSubscribeし、姿勢推定のTCPサーバへ画像送信・結果受信し、結果を`pose_estimation_result`トピックへPublishするノードの立ち上げ  
```
ros2 run pose_estimation_tcp pose_estimation_tcp --ros-args -p server_addr:=192.168.0.163 -p server_port:=54321 -p show_gui:=true
```  
