# 介绍
  本项目基于修改后的 PX4-Autopilot Iris 无人机模型，在 Gazebo 中构建仿真环境，用于验证自己编写的飞控代码、简易物体追踪和简易路径规划算法。项目涵盖 C++ 仿真飞控核心、STM32飞控代码、MAVLink 通信桥接以及视觉模块，支持简单纯软件在环（SITL）与硬件在环（HIL）两种验证模式，并在真机无人机验证实现。
## 项目文件结构
<details>
<summary>点击展开项目文件结构目录</summary>

```bash
My_FPV_Project/
├── 1Hardware/                   # 无人机硬件PCB文件
│   ├── FlightControl_board/     # 飞控板
│   └── RemoteControl_board/     # 遥控板
├── 2HIL_STM32/                  # STM32F4 飞控HIL测试代码
│   └── HIL_FlightControl/
│       ├──...                   # STM32相关配置文件以及main初始化
│       └── User/                # 核心文件
│           ├── App/             # 飞控HIL测试核心代码，包括freerots任务、飞行控制代码以及mavlink数据收发处理
│           └── Bsp/             # 移植的mavlink头文件
├── 3Simulink_Gazebo/            # Gazebo 仿真飞控软件测试
│   ├── Flight_controller/       # 核心文件
│       ├── build/               # 编译后的文件，包含名为flight_controller的可执行文件
│       ├── include/             # h文件
│       ├── src/                 # 核心cpp文件，包括main文件、传感器数据处理、飞控底层实现代码、camara视觉处理、简易路径规划以及mavlink数据收发处理文件
│       └── CMakeLists.txt       
│   └── Gazebo_Model/            # model选取并修改了PX4 iris_D435i.sdf文件，用于验证自己的飞控代码
│       ├── model/               # 包括iris_D435i、iris、gps以及自建的greebox模型文件
│       ├── worlds/              # gazebo运行世界文件
├── 4Sitl_gazebo_build/          # 编译旧版PX4的Sitl_gazebo所修改的CMakeLists.txt文件
│   ├── CMakeLists.txt           # 注释掉了不需要的文件引用和目录，以免编译报错
├── 5Pitcture/                   # 相关图片
├── 6Video/                      # 演示视频，包括简易视觉物体追踪仿真与物理实现
└── README.md                    # 本文件
```
</details>

## 效果展示
完整圆形轨迹跟踪演示视频  
[视频1](6Video/camara_track_circle.mp4)  
[视频2](6Video/drone_greenbox_track.mp4)  
仿真演示  
![运行截图](5Pitcture/gazebo仿真.jpg)
真机演示  
![运行截图](5Pitcture/追踪测试.jpg)

## 使用说明
### Gazebo仿真环境搭建与代码运行
- **Ubuntu 22.04 + Gazebo Classic 11**  
  - Ubutu：由于我虚拟机Ubuntu是18.04版本，所以我用fishros一键安装指令安装了带ros2 humble版本的ubuntu 22.04环境的docker，实测可用（我这里创建了一个docker名为Suibian，输入e启动，s进入）
  - Gazebo11：直接通过sudo apt install -y ros-humble-ros-gz安装，或者手动安装，但是官方已将其从 Ubuntu 默认源中移除，需手动添加 OSRF 官方软件源才能通过 apt 安装
- **PX4-Autopilot sitl_gazebo插件编译**
  - 我们更改的PX4-Autopilot无人机模型是旧版本的，现在的官方源码地址是最新版，我暂时不知晓新版本会不会不兼容旧版本，这里还是推荐旧版本，下载地址：
  - 由于模型需要PX4的动力学电机插件libgazebo_multirotor_base_plugin.so、libgazebo_motor_model.so以及传感器libgazebo_groundtruth_plugin.so、libgazebo_magnetometer_plugin.so等插件，所以需要在PX4-Autopilot/Tools/sitl_gazebo目录下创建build文件目录，用提供的4Sitl_gazebo_build/CMakeLists.txt覆盖原先的CMakeLists，然后输入命令进行编译,成功后会生成所需的插件和头文件
  ```bash
  cmake .. -DBUILD_GSTREAMER_PLUGIN=OFF -DMAVLINK_INCLUDE_DIRS=/root/mavlink
  make -j$(nproc)
  ```
- **软件飞控代码/3Simulink_Gazebo/Flight_controller/下的CMakeLists.txt依赖路径更改**
  - 修改sitl_gazebo/build路径为你的文件路径
  ```CMake
  set(SITL_GAZEBO_BUILD 你的sitl_gazebo/build绝对路径)
  ```
  - 修改mavlink文件路径为你下载的mavlink路径
  ```CMake
  include_directories(
    ...
    下载的mavlink路径/c_library_v2   # MAVLink 根目录
  )
  ```
- **修改/3Simulink_Gazebo/Flight_controller/代码编译**
  - 如果需要修改代码，修改完后在进入/Flight_controller/build目录编译
  ```bash
  cmake ..
  make -j$(nproc)
  ```
  - 编译完成后，会生成flight_controller可执行文件，输入下面指令运行代码
  ```bash
  ./flight_controller 2>&1 | grep -v "libprotobuf ERROR"
  ```

## 代码说明
### 无人机方向与坐标轴自定义规定
<table>
<tr>
  <td align="center"><b>1 (CW)</b><br></td>
  <td align="center"><b>↑ +X </b></td>
  <td align="center"><b>2 (CCW)</b><br></td>
</tr>
<tr>
  <td align="center">← +Y </td>
  <td></td>
  <td align="center">Y- →(机头前进方向)</td>
</tr>
<tr>
  <td align="center"><b>3 (CCW)</b><br></td>
  <td align="center"><b>↓ -X </b></td>
  <td align="center"><b>0 (CCW)</b><br></td>
</tr>
</table>

### 2HIL_STM32
STM32F4 HIL测试的代码，工作流程：
- PC端：gazebo启动仿真，Flight_controller代码在main.cpp中修改模式mode为HIL模式并启动运行，这时Flight_controller会订阅gazebo传感器发布的数据，包括imu的6轴数据、mag磁力计三轴数据、气压计数据以及gps的三轴坐标数据，之后将数据打包给MAVLINk。
- STM32F4：通过USB虚拟串口CDC与PC虚拟机进行MAVLINK通信，将接收的数据进行处理，接着进入飞行控制部分，包括姿态控制、位置控制以及简易路径规划位置控制，最后将计算出的四个电机速度打包给MAVLINK传回PC，STM32工程主要文件及说明如下：
    - usbd_cdc_if.c:修改CDC_Receive_FS函数
    - main.c:进行初始化
    - MyTask.c:创建静态FreeRTOS任务与静态队列，三个任务中飞控任务为最高优先级
      - StartCDCReceiveTask任务：接收MAVLINK传感器数据，通过队列传递给飞控任务。
      - FlightControlTask任务：队列接收数据，进行传感器数据处理以及飞行控制（200Hz）计算，将计算结果通过队列传给发送任务。
      - SendActuatorTask任务：队列接收计算结果，MAVLINK发送数据。
    - /User/App下的其他文件: 包括全局变量、传感器数据处理、姿态控制、位置控制以及规划文件
  
### 3Simulink_Gazebo


## 真机介绍
无人机图片展示
![无人机1](5Pitcture/无人机1.jpg)
![无人机2](5Pitcture/无人机2.jpg)
### 硬件组成
- ESC电调：由四个单独的AM32电调组成，AM32配置如下：
  ![AM32电调配置](5Pitcture/AM32配置.png)
- 飞控板：由STM32F4最小系统板、各个传感器以及DCDC降压模块组成
    - 飞控PCB板：[PCB1](1Hardware/FlightControl_board/Flight_Control_V3.eprj)
    - 传感器：MPU6050模块、NRF24L01无线通信模块、HMC5883L磁力计、ThoneFlow-3901U光流传感器构成
- 遥控板：[PCB2](1Hardware/RemoteControl_board/Remote_Board.eprj)
- 视觉识别小主机：由泰山派RK3566开发板、USB摄像头以及电源模块构成
  
  
