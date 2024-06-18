# 内参标定
## 数据目录
```
data_path
├── camera_video_0.avi
├── camera_video_1.avi
├── camera_video_2.avi
├── camera_video_3.avi
├── camera_video_6.avi
└── camera_video_7.avi
```
## 编译运行
```bash
cmake .
make calib_intrin
./calib_intrin -d /home/touch/data/0730_intrin_test -n 0 -n 1 -n 2 -n 3 -n 4 -n 5 -n 6 
```
* `-d` 为视频文件夹路径
* `-n` 为摄像机编号，样例中使用了6个相机，若其他数量可以直接增减
## 运行结果

```
data_path
├── ...
├── camera_0_intrin.yaml
├── camera_1_intrin.yaml
├── camera_2_intrin.yaml
├── camera_3_intrin.yaml
├── camera_6_intrin.yaml
├── camera_7_intrin.yam
└── ...
```
# 环视标定

## 数据目录
```
data_path
├── airview
│   └──  airview.jpg
├── camera_0_intrin.yaml
├── camera_1_intrin.yaml
├── camera_2_intrin.yaml
├── camera_3_intrin.yaml
├── camera_6_intrin.yaml
├── camera_7_intrin.yaml
├── camera_video_0.avi
├── camera_video_1.avi
├── camera_video_2.avi
├── camera_video_3.avi
├── camera_video_6.avi
└── camera_video_7.avi
```

## 编译运行
```bash
cmake .
make pano_calib
./pano_calib -d /home/touch/data/0724_pano_calib_1 -n 0 -n 1 -n 2 -n 3 -n 6 -n 7 -b 0.2
```
* `-d` 为视频文件夹路径
* `-n` 为摄像机编号，样例中使用了6个相机，若其他数量可以直接增减
* `-b` 为反畸变参数，通常使用`0.2`即可
## 运行结果
```
data_path
├── ...
├── camera_0_homography.yaml
├── camera_1_homography.yaml
├── camera_2_homography.yaml
├── camera_3_homography.yaml
├── camera_6_homography.yaml
├── camera_7_homography.yaml
└── ...
```
需要使用的相机对应的Homography矩阵保存在文件中，拷贝到环视拼接工程的`./parameters/yamls`目录下进行使用
# 前视标定
## 数据目录
```
data_path
├── camera_5_intrin.yaml
├── camera_8_intrin.yaml
├── camera_9_intrin.yaml
├── camera_video_5.avi
├── camera_video_8.avi
└── camera_video_9.avi
```
例子中5为中间的相机

## 编译运行
```bash
cmake.
make calib_cameras
make get_poses
./get_poses -d /home/touch/data/0730_front_calib_cameras -n 5 -n 8 -n 9 -s 0.06 # 命令1
./calib_cameras -d /home/touch/data/0730_front_calib_cameras -n 5 -n 9  # 命令2
./calib_cameras -d /home/touch/data/0730_front_calib_cameras -n 5 -n 8  
```
### 命令1
* `-s` 表示标定使用的tag尺寸(正方形边长，单位为米)

结果如下：
```
data_path
├── ...
├── 5_mask.png
├── 5_undistort.avi
├── 8_mask.png
├── 8_undistort.avi
├── 9_mask.png
├── 9_undistort.avi
├── camera_5_intrin_undistort.yaml
├── camera_8_intrin_undistort.yaml
├── camera_9_intrin_undistort.yaml
├── camera_calib
│   ├── 5-0.png
│   ├── 5-1.png
│   ├── ....
│   └── poses
│       ├── 5-0.png.yaml
│       ├── 5-1.png.yaml
│       └── ....
└── ...
```

### 命令2
`-n 5 -n 9` 分别为中间相机和旁边相机，保持中间相机在前的顺序
## 运行结果
```
data_path
├── ...
├── camera_extrin_5_and_8.yaml
└── ...
```
最终将`camera_extrin_5_and_8.yaml`和`camera_extrin_5_and_9.yaml`拷贝到前视拼接工程的`./example/yamls`目录下使用

# 地面标定
## 数据目录
```
data_path
├── camera_5_intrin.yaml
└── camera_video_5.avi
```
## 编译运行
```bash
cmake .
make get_poses
make calib_ground
./get_poses -d /home/touch/data/0730_front_calib_cameras -n 5 -s 0.675 # 命令1
./calib_ground -d /home/touch/data/0731_front_calib_test -n 5 -t 56 # 命令2
```
命令1和前视标定类似，从`5-0.png.yaml`或者其他文件中选择一个tag序号作为后续的参考序号
### 命令2
`-t`表示参考的tag序号

运行结果如下
```
data_path
├── ...
└── ground.yaml
```
结果保存在`ground.yaml`中，拷贝到前视拼接工程的`./example/yamls`目录下使用
