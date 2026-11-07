# 相机内参标定例程

这个小程序直接打开相机，实时检测棋盘格并采集有效照片，估计相机内参和畸变系数，最后保存为 OpenCV YAML 文件。它是独立 CMake 工程，不会改动主瞄准程序。

## 采集标定照片

- 使用最终运行时的相机、镜头、对焦状态和分辨率。
- 棋盘格保持平整，完整出现在画面中且角点清晰；移动棋盘覆盖画面中心、四角和边缘，并包含不同距离和倾斜角度。
- 建议采集 15–30 张不同姿态的照片。不要连续拍摄完全相同的画面。
- 参数中的列、行是棋盘的**内角点数**，不是方格数。比如 10 列、7 行方格对应 9 列、6 行内角点。
- 准确测量一个小方格的边长，单位为毫米。

## 构建和运行

在仓库根目录执行：

```powershell
cmake -S .\camera_calibration -B .\camera_calibration\build
cmake --build .\camera_calibration\build --config Release
.\camera_calibration\build\Release\camera_calibration.exe `
  9 6 25 .\camera_intrinsics.yml 0 1280 720
```

参数依次是：内角点列数、内角点行数、方格边长（毫米）、输出 YAML 文件、相机编号、图像宽度、图像高度。后三项可省略，默认相机编号为 `0`、分辨率为 `1280×720`。程序和主瞄准程序一样请求 MJPG 和 30 FPS，并在采集首帧时核对实际分辨率；相机不支持请求的尺寸时会停止，不会生成不同分辨率的标定结果。若可执行文件不在 `Release` 子目录，请在 `camera_calibration\build` 下查找。

运行后会打开相机预览窗口：

- **空格**：检测到完整棋盘时采集当前帧；画面未检测到棋盘时不会计数。
- **U**：撤销最近一次采集。
- **C**：使用当前有效照片标定并保存结果；至少需要 5 张有效照片，建议 15 张以上。
- **Q** 或 **Esc**：退出。

窗口和控制台会显示有效照片数量。标定文件会保存到命令行指定的路径；请确保其父目录已经存在。

## 输出与主程序参数

YAML 中的 `camera_matrix` 为：

```text
[ fx  0  cx
   0 fy  cy
   0  0   1 ]
```

将其填入主程序：

```cpp
kCamIntrinsics.fx = fx;
kCamIntrinsics.fy = fy;
kCamIntrinsics.cx = cx;
kCamIntrinsics.cy = cy;
kCamIntrinsics.distCoeffs = {k1, k2, p1, p2, k3};
```

这个例程输出标准 5 项畸变系数 `[k1, k2, p1, p2, k3]`。运行时图像尺寸要与 YAML 的 `image_width`、`image_height` 相同。当前瞄准程序直接将内参和畸变系数用于 PnP，不要再对输入帧重复去畸变。

程序输出每张有效照片的重投影 RMS 和整体误差。若误差明显偏大、参数异常或矫正后的直线仍明显弯曲，应检查角点数、方格尺寸、照片清晰度及覆盖范围后重新拍摄标定。
