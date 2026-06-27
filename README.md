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
仿真演示
![运行截图](5Pitcture/gazebo仿真.jpg)
真机演示
![运行截图](5Pitcture/追踪测试.jpg)

## 使用说明
### Gazebo仿真环境搭建
- **Ubuntu 22.04 + Gazebo Classic 11**  
  - Ubutu：由于我虚拟机Ubuntu是18.04版本，所以我用fishros一键安装指令安装了带ros2 humble版本的ubuntu 22.04环境的docker，实测可用（我这里创建了一个docker名为Suibian，输入e启动，s进入）
  - Gazebo11：官方已将其从 Ubuntu 默认源中移除，需手动添加 OSRF 官方软件源才能通过 apt 安装
- **PX4-Autopilot sitl_gazebo插件编译**
  - 我们更改的PX4-Autopilot无人机模型是旧版本的，现在的官方源码地址是最新的，我暂时不知晓会不会新版本会不会不兼容旧版本，这里还是推荐旧版本，下载地址：
  - 由于模型需要PX4的动力学电机插件libgazebo_multirotor_base_plugin.so、libgazebo_motor_model.so以及传感器插件libgazebo_groundtruth_plugin.so、libgazebo_magnetometer_plugin.so等插件，所以需要在PX4-Autopilot/Tools/sitl_gazebo目录下创建build文件目录，用提供的4Sitl_gazebo_build下的CMakeLists.txt覆盖原先的，然后输入命令进行编译,成功后会生成所需的插件和头文件
  ```bash
  cmake .. -DBUILD_GSTREAMER_PLUGIN=OFF -DMAVLINK_INCLUDE_DIRS=/root/mavlink
  make -j$(nproc)
  ```
  
  
