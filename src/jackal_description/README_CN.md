# Jackal描述

本软件包包含Jackal机器人的网格模型和URDF文件，以及其支持的传感器和挂载支架。

# Jackal 载荷配置

## Microstrain IMU
```bash
export JACKAL_IMU_MICROSTRAIN=1
```
###### 启动参数
```bash
export JACKAL_IMU_MICROSTRAIN_NAME="microstrain"
```
###### 描述参数
```bash
export JACKAL_IMU_MICROSTRAIN_LINK="microstrain_link"
export JACKAL_IMU_MICROSTRAIN_PARENT="base_link"
export JACKAL_IMU_MICROSTRAIN_OFFSET="-0.139 0.096 0.100"
export JACKAL_IMU_MICROSTRAIN_RPY="3.14159 0 -1.5707"
```

## 2D激光雷达
#### 主要激光雷达
```bash
export JACKAL_LASER=1
export JACKAL_LASER_MODEL=lms1xx # 或 tim551 或 ust10 或 utm30
```
###### 启动参数
```bash
export JACKAL_LASER_TOPIC="front/scan"
export JACKAL_LASER_HOST="192.168.131.20"
```
###### 描述参数
```bash
export JACKAL_LASER_TOWER=1
export JACKAL_LASER_MOUNT="front"
export JACKAL_LASER_PREFIX="front"
export JACKAL_LASER_PARENT="front_mount"
export JACKAL_LASER_MOUNT_TYPE="upright" # 或 "inverted"
export JACKAL_LASER_OFFSET="0 0 0"
export JACKAL_LASER_RPY="0 0 0"
```
#### 次要激光雷达
```bash
export JACKAL_LASER_SECONDARY=1
export JACKAL_LASER_SECONDARY_MODEL="lms1xx" # 或 "tim551" 或 "ust10" 或 "utm30"
```
###### 启动参数
```bash
export JACKAL_LASER_SECONDARY_HOST="192.168.131.21"
export JACKAL_LASER_SECONDARY_TOPIC="rear/scan"
```
###### 描述参数
```bash
export JACKAL_LASER_SECONDARY_TOWER=1
export JACKAL_LASER_SECONDARY_MOUNT="rear"
export JACKAL_LASER_SECONDARY_PREFIX="rear"
export JACKAL_LASER_SECONDARY_PARENT="rear_mount"
export JACKAL_LASER_SECONDARY_OFFSET="0 0 0"
export JACKAL_LASER_SECONDARY_RPY="0 0 3.13159"
```

## 3D激光雷达
```bash
export JACKAL_LASER_3D=1
```
###### 启动参数
```bash
export JACKAL_LASER_3D_HOST="192.168.131.20"
export JACKAL_LASER_3D_TOPIC="mid/points"
```
###### 描述参数
```bash
export JACKAL_LASER_3D_TOWER=1
export JACKAL_LASER_3D_MOUNT="mid"
export JACKAL_LASER_3D_PREFIX="mid"
export JACKAL_LASER_3D_PARENT="mid_mount"
export JACKAL_LASER_3D_MODEL="vlp16" # 或 "hdl32e"
export JACKAL_LASER_3D_OFFSET="0 0 0"
export JACKAL_LASER_3D_RPY="0 0 0"
```

## NAVSAT（导航卫星）
```bash
export JACKAL_NAVSAT=1
```
###### 启动参数
```bash
export JACKAL_NAVSAT_PORT="/dev/clearpath/gps"
export JACKAL_NAVSAT_BAUD=57600
export JACKAL_NAVSAT_RTK=0
export JACKAL_NAVSAT_RTK_DEVICE=wlan0
export JACKAL_NAVSAT_RTK_BAUD=57600
```
###### 描述参数
```bash
export JACKAL_NAVSAT_TOWER=1
export JACKAL_NAVSAT_HEIGHT=0.1 # 单位：米
export JACKAL_NAVSAT_MOUNT="rear"
export JACKAL_NAVSAT_PREFIX="rear"
export JACKAL_NAVSAT_PARENT="rear_mount"
export JACKAL_NAVSAT_MODEL="smart6" # 或 "smart7"
export JACKAL_NAVSAT_OFFSET="0 0 0"
export JACKAL_NAVSAT_RPY="0 0 0"
```

## Pointgrey Flea3相机
```bash
export JACKAL_FLEA3=1
```
###### 启动参数
```bash
export JACKAL_FLEA3_SERIAL=0
export JACKAL_FLEA3_FRAME_RATE=30
export JACKAL_FLEA3_CALIBRATION=0
```
###### 描述参数
```bash
export JACKAL_FLEA3_TOWER=1
export JACKAL_FLEA3_TILT=0.5236
export JACKAL_FLEA3_NAME="front"
export JACKAL_FLEA3_MOUNT="front"
export JACKAL_FLEA3_PREFIX="front"
export JACKAL_FLEA3_PARENT="front_mount"
export JACKAL_FLEA3_OFFSET="0 0 0"
export JACKAL_FLEA3_RPY="0 0 0"
```

## 立体Pointgrey Flea3相机
```bash
export JACKAL_STEREO_FLEA3=1
```
###### 启动参数
```bash
export JACKAL_FLEA3_FRAME_RATE=30
export JACKAL_FLEA3_LEFT_SERIAL=0
export JACKAL_FLEA3_LEFT_CALIBRATION=0
export JACKAL_FLEA3_RIGHT_SERIAL=0
export JACKAL_FLEA3_RIGHT_CALIBRATION=0
```
###### 描述参数
```bash
export JACKAL_STEREO_SEPERATION=0.16
export JACKAL_FLEA3_TILT="0.5236"
export JACKAL_FLEA3_MOUNT="front"
export JACKAL_FLEA3_PREFIX="front"
export JACKAL_FLEA3_PARENT="front_mount"
export JACKAL_FLEA3_LEFT_NAME="front/left"
export JACKAL_FLEA3_RIGHT_NAME="front/right"
export JACKAL_FLEA3_OFFSET="0 0 0"
export JACKAL_FLEA3_RPY="0 0 0"
```

## Bumblebee2相机
```bash
export JACKAL_BB2=1
```
###### 启动参数
```bash
export JACKAL_BB2_SERIAL=0
export JACKAL_BB2_CALIBRATION=0
```
###### 描述参数
```bash
export JACKAL_BB2_TILT=0
export JACKAL_BB2_TOWER=1
export JACKAL_BB2_NAME="front"
export JACKAL_BB2_MOUNT="front"
export JACKAL_BB2_PREFIX="front"
export JACKAL_BB2_PARENT="front_mount"
export JACKAL_BB2_OFFSET="0 0 0"
export JACKAL_BB2_RPY="0 0 0"
```

## Flir Blackfly相机
```bash
export JACKAL_BLACKFLY=1
```
###### 启动参数
```bash
export JACKAL_BLACKFLY_SERIAL=0
export JACKAL_BLACKFLY_DEVICE="USB" # 或 "GigE"
export JACKAL_BLACKFLY_ENCODING="BayerRGB"
export JACKAL_BLACKFLY_FRAMERATE=30
```
###### 描述参数
```bash
export JACKAL_BLACKFLY_PREFIX="front_camera"
export JACKAL_BLACKFLY_PARENT="front_mount"
export JACKAL_BLACKFLY_OFFSET="0 0 0"
export JACKAL_BLACKFLY_RPY="0 0 0"
```

## 前后辅助挡泥板
```bash
export JACKAL_FRONT_ACCESSORY_FENDER=1
export JACKAL_REAR_ACCESSORY_FENDER=1
export JACKAL_FRONT_FENDER_UST10=1
export JACKAL_REAR_FENDER_UST10=1
```
###### 启动参数
```bash
export JACKAL_FRONT_LASER_TOPIC=front/scan
export JACKAL_FRONT_LASER_HOST="192.168.131.20"
export JACKAL_REAR_LASER_TOPIC=rear/scan
export JACKAL_REAR_LASER_HOST="192.168.131.21"
```

## 装饰配件
仅用于描述的配件（即没有驱动程序）。
###### Wibotic Q充电保险杠：
```bash
export JACKAL_WIBOTIC_BUMPER=1
```
###### 背包式计算机外壳：
```bash
export JACKAL_ARK_ENCLOSURE=1


找到具有 1 个许可证类型的类似代码