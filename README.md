# 说明

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
|       ├──...                   # STM32相关配置文件以及main初始化
|       └── User/                # 核心文件
|           ├── App/             # 飞控HIL测试核心代码，包括freerots任务、飞行控制代码以及mavlink数据收发处理
|           └── Bsp/             # 移植的mavlink头文件
├── 3Simulink_Gazebo/            # Gazebo 仿真飞控软件测试
│   ├── Flight_controller/       # 核心文件
|       ├── build/               # 编译后的文件，包含名为flight_controller的可执行文件
|       ├── include/             # h文件
|       ├── src/                 # 核心cpp文件，包括main文件、传感器数据处理、飞控底层实现代码、camara视觉处理、简易路径规划以及mavlink数据收发处理文件
|       └── CMakeLists.txt       
│   └── Gazebo_Model/
|       ├── model/               # 
|       ├── worlds/              # 
├── 4Sitl_gazebo_build/          # 
│   ├── CMakeLists.txt           # 编译旧版PX4的Sitl_gazebo所修改的CMakeLists.txt文件，注释掉了不需要的文件引用和目录，以免编译报错
├── 5Pitcture/
├── 6Video/
└── README.md                    # 本文件
```
</details>
