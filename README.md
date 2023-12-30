

## bag数据处理

* 使用 `bag2img.py` 对从bag中提取彩色图像和深度图像，其中 `--bag_path` 表示bag的路径
``` shell
$ python3 bag2img.py --bag_path /path/to/bag

# example:
# python3 bag2img.py --bag_path /home/lg/rosbags/map-1230-room_converted.bag
```

* 执行该指令后将在bag所在目录下生成一个同名文件夹，包含如下内容：
```
├─depth
│  ├─xxxxxxxxxx.xxxxxx.png  # 深度图像，以时间戳命名
├─depth-stamp.txt           # 深度图像的时间戳列表
├─rgb
│  ├─xxxxxxxxxx.xxxxxx.png  # 彩色图像，以时间戳命名
├─rgb-stamp.txt             # 深度图像的时间戳列表
├─associations.txt          # 彩色图像与深度图像的对应关系，格式为（时间戳 彩色图像 深度图像）
```

## 生成轨迹数据

* 将slam生成的轨迹数据保存为文本文件，参考 `CameraTrajectory.txt`，其格式为：
```
timestamp1 x1 y1 z1 qx1 qy1 qz1 qw1
timestamp2 x2 y2 z2 qx2 qy2 qz2 qw2
timestamp3 x3 y3 z3 qx3 qy3 qz3 qw3
......
```

## 重建点云

* 修改 `rgbd_mapping.cc` 中 `T_body_rgb` 的值。由于我的slam算法以imu作为body坐标系，所以我将 `T_body_rgb` 设置为rgb图像到imu的变换，请根据所使用slam算法的body坐标系修改 `T_body_rgb`。

* 编译代码
``` shell
$ mkdir build
$ cd build
$ cmake ..
$ make
```

* 执行 `RGBD_Mapping` 进行见图。其中，`trajectory_path` 为轨迹数据文件的路径；`data_dir` 为处理bag数据时生成的文件夹；`pcd_save_path` 为重建点云的保存路径，重建的点云以pcd格式保存；`depth_sample_interval`为深度图像的采样间隔，增大采样间隔可以减少点的数量，减小采样间隔使重建点云更加精细
``` shell
$ ./RGBD_Mapping trajectory_path data_dir pcd_save_path depth_sample_interval

# example
# ./RGBD_Mapping /home/lg/mapping/CameraTrajectory.txt /home/lg/rosbags/map-1230-room_converted map.pcd 8
```
