# Multi 模式完整链路说明

本文档说明当前 `RUN_MODE = "multi"` 时，主机端 `CarControlCode/main.py` 与视觉端 `VisionCode/sekuai.py` 的完整通信链路、执行流程和关键配置。

## 相关文件

- 主机控制入口：`CarControlCode/main.py`
- 主机全局配置：`CarControlCode/robot_config.py`
- 视觉识别程序：`VisionCode/sekuai.py`
- 底盘控制：`CarControlCode/chassis_control.py`
- 机械臂控制：`CarControlCode/arm_control.py`
- 舵机功能封装：`CarControlCode/servo_control.py`

## 运行前提

主机端需要在 `CarControlCode/robot_config.py` 中设置：

```python
RUN_MODE = "multi"
```

视觉端需要运行 `VisionCode/sekuai.py`。主机与视觉端通过 UART 通信，主机配置为：

```python
CAMERA_UART_ID = 1
CAMERA_UART_BAUD = 115200
CAMERA_UART_TX = 5
CAMERA_UART_RX = 6
```

视觉端 `sekuai.py` 使用：

```python
device = "/dev/ttyS0"
serial = uart.UART(device, 115200)
```

## 上电初始化

主机启动后会先执行 `reset_all_servos()`：

1. 底盘 6 个转向舵机回中。
2. 机械臂回初始位。
3. 相机舵机转到 `CAMERA_INIT_ANGLE_DEG`，当前为 `0 deg`。
4. 预留舵机初始化。当前夹爪舵机 ID 为 `15`，初始角来自 `RESERVE_SERVO_INIT_ANGLE_DEG = {15: 100}`。
5. 等待 2 秒，保证复位动作完成。

随后启动相机 UART 接收线程。该线程会把收到的文本追加到 `camera_data["value"]` 短缓冲中，避免二维码任务被后续视觉帧覆盖。

## 进入 multi_loop

当 `RUN_MODE == "multi"` 时，主机会初始化 PS2 手柄接收器，并进入 `multi_loop(ps2)`。

进入 `multi_loop()` 后，主机会执行：

```python
rover.prepare()
```

该调用会：

1. 底盘转向舵机回中。
2. 使能四个电机。

此后 PS2 可以继续控制小车移动。

## 二维码任务链路

二维码任务完全由视觉端自动完成，不需要 L3 或其他 PS2 按键触发扫码确认。

视觉端 `sekuai.py` 在 `task_loaded == False` 时持续扫描二维码：

```python
qrcodes = img.find_qrcodes()
```

二维码内容格式为：

```text
颜色1 颜色2 颜色3 数量1 数量2 数量3
```

可用颜色为：

```text
red pink blue purple yellow
```

示例：

```text
red blue yellow 2 1 3
```

视觉端解析后会通过 UART 发送原始任务字符串：

```text
red blue yellow 2 1 3
```

主机端 `multi_loop()` 持续监听 `camera_data["value"]`。当收到非 `sx`、非 `ln` 的任务字符串后，调用 `parse_qrcode_task()` 解析并展开为抓取队列：

```python
["red", "red", "blue", "yellow", "yellow", "yellow"]
```

解析成功后，主机会向相机发送：

```text
ok
```

视觉端收到 `ok` 后只打印确认信息，不改变任务逻辑。

## PS2 控制

multi 模式下 PS2 主要用于手动驾驶和触发执行。

- `R3`：开始执行 multi 抓取/放置流程。
- `L3`：在 R3 执行前将相机转到 `CAMERA_L3_ANGLE_DEG = -77.5 deg`。
- `R1`：停车。
- `SELECT`：退出 multi 模式。
- `R2 + 右摇杆 X`：原地转向。
- 常规驾驶：右摇杆 Y 控制前后，左摇杆 X 控制转向。

如果电机未使能，主机会拦截行驶命令并打印提示。

## R3 后的入口对齐流程

用户按下 `R3` 后，主机先停车：

```python
rover.stop()
```

如果二维码任务还未加载，则不会执行抓取，只提示等待视觉端自动识别二维码。

如果任务已加载，主机会调用：

```python
execute_multi_grab_placeholder(task_queue)
```

该函数会先把相机转到 `CAMERA_L3_ANGLE_DEG = -77.5 deg` 进入观察状态，然后执行入口自动对齐：

```python
align_multi_left_bottom_anchor()
```

### 入口对齐目标

入口对齐的目标是：

> 将相机画面中 3x3 色块阵列的左下角色块中心，对齐到画面中心红框中心。

这样小车当前位置会被视为第一列起点，后续三列遍历从这里开始。

### 入口对齐通信

主机发送：

```text
multi_anchor
```

视觉端收到后：

1. 使用 5 种颜色阈值识别当前画面所有色块。
2. 将所有候选色块按中心 `y` 从大到小排序，取最下面一行的 3 个候选。
3. 在这 3 个候选中取中心 `x` 最小的色块，作为 3x3 左下角基准色块。
4. 将本帧画面、中心红框、所有识别到的色块外接框、颜色标签和中心点保存到 `./debug`。
5. 返回该色块中心相对红框中心的偏差。

返回格式为：

```text
md color dx dy
```

如果未识别到色块，返回：

```text
md none
```

其中：

- `dx = blob_center_x - target_center_x`
- `dy = blob_center_y - target_center_y`

### 入口调试图片

每次视觉端收到 `multi_anchor`，都会输出一张调试图片：

```text
./debug/multi_anchor_序号_候选数量.jpg
```

例如：

```text
./debug/multi_anchor_0001_09.jpg
```

图片中会绘制：

- 画面中心红框。
- 5 种阈值识别到的所有候选色块外接框。
- 每个候选色块的颜色标签和中心坐标。
- 每个候选色块的中心十字。

这张图用于确认是否稳定识别出 3x3 的 9 个方块，以及每个方块颜色分类是否正确。

### 入口对齐判定

主机复用 `multi_detection_centered()` 判断是否对齐：

```python
abs(dx) <= MULTI_CENTER_TOLERANCE_PX
abs(dy) <= MULTI_CENTER_TOLERANCE_PX
```

当前配置：

```python
MULTI_CENTER_TOLERANCE_PX = 8
MULTI_ENTRY_ALIGN_TIMEOUT_MS = 15000
```

如果未对齐，主机会调用 `track_camera_target()` 控制底盘微调，并继续请求 `multi_anchor`。超过 `MULTI_ENTRY_ALIGN_TIMEOUT_MS` 后仍未对齐，则本次 R3 执行失败并退出。

## 三列搜索流程

入口对齐成功后，主机开始遍历 `task_queue`。

每处理一个目标颜色时，主机都会尽量从当前第一列起点开始搜索：

1. 当前列检测。
2. 如果颜色匹配，先对该目标色块的 `dx/dy` 再次闭环微调。
3. 微调到目标色块中心与红框中心重合后，执行抓取/放置。
4. 如果颜色不匹配，横移到下一列。
5. 最多检查 3 列。

列数配置：

```python
MULTI_COLUMN_COUNT = 3
```

当前列检测使用命令：

```text
multi_detect
```

视觉端收到后，会从当前画面所有色块中选择最靠近中心红框的色块并返回：

```text
md color dx dy
```

主机先检查 `color` 是否等于当前目标颜色。颜色匹配后，主机会继续请求 `multi_detect` 并调用 `track_camera_target()` 微调底盘，直到：

```python
abs(dx) <= MULTI_CENTER_TOLERANCE_PX
abs(dy) <= MULTI_CENTER_TOLERANCE_PX
```

微调过程中如果最近中心色块变成其他颜色，本次抓取会失败退出，避免对齐错误色块后误抓。

## 横移逻辑

横移由主机函数 `multi_move_horizontal(direction_steps)` 完成。

每横移一列时执行：

```python
rover.drive(
    MULTI_HORIZONTAL_SPEED_RAD_S * direction,
    MULTI_HORIZONTAL_STEER_DEG,
)
time.sleep_ms(MULTI_COLUMN_MOVE_MS)
rover.stop()
time.sleep_ms(150)
```

当前配置：

```python
MULTI_HORIZONTAL_STEER_DEG = 90.0
MULTI_HORIZONTAL_SPEED_RAD_S = 0.6
MULTI_COLUMN_MOVE_MS = 900
```

`MULTI_COLUMN_MOVE_MS` 的单位是毫秒。它决定从一列横移到下一列的持续时间，需要实车标定。如果轮半径是 5 cm，目标横移距离是 8 cm，理论值为约 `2667 ms`，但实车会受到打滑和地面摩擦影响。

## 同色多次抓取的行顺序

每种颜色有 3 个同色块，当前同色多次抓取顺序为：

```text
第三行 -> 第二行 -> 第一行
```

实现逻辑：

```python
grab_pose_index = len(MULTI_GRAB_ROW_POSES) - 1 - row_index
```

其中 `row_index` 是该颜色已经抓取过的次数：

- 第 1 次抓某颜色：`row_index = 0`，使用第三行姿态。
- 第 2 次抓某颜色：`row_index = 1`，使用第二行姿态。
- 第 3 次抓某颜色：`row_index = 2`，使用第一行姿态。

如果某颜色需要抓取超过 3 个，会触发越界检查并失败退出。

## 单次抓取/放置动作

命中目标颜色后，主机调用：

```python
execute_multi_single_grab_place_placeholder(
    color,
    column_index,
    row_index,
    place_index,
)
```

动作顺序如下：

1. `rover.stop()` 停车。
2. 相机保持观察角 `CAMERA_L3_ANGLE_DEG = -77.5 deg`。
3. 相机舵机转到 `CAMERA_INIT_ANGLE_DEG = 0 deg`。
4. 等待 `ARM_AUTO_ACTION_DELAY_MS`。
5. 机械臂移动到对应抓取行的四关节绝对姿态 `[roll, pitch1, pitch2, pitch3]`。
6. 等待 `ARM_AUTO_ACTION_DELAY_MS`。
7. 夹爪闭合，执行抓取。
8. 等待 `ARM_AUTO_ACTION_DELAY_MS`。
9. 机械臂移动到当前放置序号对应的四关节绝对姿态 `[roll, pitch1, pitch2, pitch3]`。
10. 等待 `ARM_AUTO_ACTION_DELAY_MS`。
11. 夹爪打开，释放物块。
12. 等待 `ARM_AUTO_ACTION_DELAY_MS`。
13. 执行复位。

当前动作间隔：

```python
ARM_AUTO_ACTION_DELAY_MS = 3000
```

## 单次抓放后的复位

单次抓放动作结束后，无论成功还是中途失败，都会进入 `finally` 并调用：

```python
reset_multi_action_servos()
```

复位内容：

1. 底盘 6 个转向舵机回中。
2. 机械臂回初始位。
3. 相机回到 `CAMERA_INIT_ANGLE_DEG`，当前是 `0 deg`。
4. 夹爪打开。

如果后面还有任务，主机会再把相机转回 `CAMERA_L3_ANGLE_DEG = -77.5 deg` 进入下一轮观察；如果全部任务完成，相机保持 `0 deg`。

## 抓取姿态配置

multi 抓取姿态在 `robot_config.py` 中配置，每一行是四个绝对角：

```python
[roll, pitch1, pitch2, pitch3]
```

当前配置：

```python
MULTI_GRAB_ROW1_ROLL_DEG = 0.8
MULTI_GRAB_ROW1_PITCH1_DEG = -64.9
MULTI_GRAB_ROW1_PITCH2_DEG = -109.8
MULTI_GRAB_ROW1_PITCH3_DEG = 0.0

MULTI_GRAB_ROW2_ROLL_DEG = 0.7
MULTI_GRAB_ROW2_PITCH1_DEG = -65.5
MULTI_GRAB_ROW2_PITCH2_DEG = -99.0
MULTI_GRAB_ROW2_PITCH3_DEG = 7.0

MULTI_GRAB_ROW3_ROLL_DEG = 1.2
MULTI_GRAB_ROW3_PITCH1_DEG = -72.0
MULTI_GRAB_ROW3_PITCH2_DEG = -65.7
MULTI_GRAB_ROW3_PITCH3_DEG = -28.8
```

注意：虽然变量名是 `ROW1/ROW2/ROW3`，实际抓取同色块时按 `ROW3 -> ROW2 -> ROW1` 使用。

## 放置姿态配置

放置姿态也按四个绝对角配置：

```python
[roll, pitch1, pitch2, pitch3]
```

放置序号由 `place_index` 决定，也就是当前执行的是任务队列中的第几个物体：

- 第 1 个物体使用 `MULTI_PLACE_1_*`
- 第 2 个物体使用 `MULTI_PLACE_2_*`
- 依此类推，最多 6 个

当前配置：

```python
MULTI_PLACE_1_ROLL_DEG = 1.3
MULTI_PLACE_1_PITCH1_DEG = -2.5
MULTI_PLACE_1_PITCH2_DEG = 99.9
MULTI_PLACE_1_PITCH3_DEG = 77.1

MULTI_PLACE_2_ROLL_DEG = 0.3
MULTI_PLACE_2_PITCH1_DEG = -1.3
MULTI_PLACE_2_PITCH2_DEG = 92.3
MULTI_PLACE_2_PITCH3_DEG = 73.8

MULTI_PLACE_3_ROLL_DEG = 0.5
MULTI_PLACE_3_PITCH1_DEG = 16.2
MULTI_PLACE_3_PITCH2_DEG = 71.7
MULTI_PLACE_3_PITCH3_DEG = 83.9

MULTI_PLACE_4_ROLL_DEG = -14.2
MULTI_PLACE_4_PITCH1_DEG = -1.2
MULTI_PLACE_4_PITCH2_DEG = 93.9
MULTI_PLACE_4_PITCH3_DEG = 82.8

MULTI_PLACE_5_ROLL_DEG = 12.1
MULTI_PLACE_5_PITCH1_DEG = -0.1
MULTI_PLACE_5_PITCH2_DEG = 94.0
MULTI_PLACE_5_PITCH3_DEG = 82.0

MULTI_PLACE_6_ROLL_DEG = -10.7
MULTI_PLACE_6_PITCH1_DEG = 17.2
MULTI_PLACE_6_PITCH2_DEG = 69.5
MULTI_PLACE_6_PITCH3_DEG = 86.3
```

## 夹爪配置

当前夹爪配置：

```python
GRIPPER_SERVO_ID = 15
GRIPPER_OPEN_ANGLE_DEG = 100.0
GRIPPER_CLOSED_ANGLE_DEG = 50
```

`run_gripper_grab()` 使用 `GRIPPER_CLOSED_ANGLE_DEG`。

`run_gripper_release()` 使用 `GRIPPER_OPEN_ANGLE_DEG`。

如果实车最紧闭合角需要 `52.8 deg`，应将 `GRIPPER_CLOSED_ANGLE_DEG` 调整为 `52.8`。

## 视觉颜色阈值

视觉端当前使用 5 组 LAB 阈值：

```python
thresholds_red = [[30, 43, 30, 40, 9, 18]]
thresholds_pink = [[56, 64, 18, 25, -1, -7]]
thresholds_blue = [[36, 46, -2, 5, -45, -36]]
thresholds_purple = [[24, 37, 13, 19, -35, -44]]
thresholds_yellow = [[54, 64, -6, 1, 39, 57]]
```

这些值与现场光照、相机曝光、物块材质高度相关。若识别不稳定，应优先重新标定这些阈值。

## 红框与中心判定

视觉端红框配置：

```python
TARGET_LEFT = 305
TARGET_TOP = 225
TARGET_WIDTH = 30
TARGET_HEIGHT = 30
```

红框中心为 `(320, 240)`。

multi 入口对齐和中心判定使用主机端：

```python
MULTI_CENTER_TOLERANCE_PX = 8
```

如果实车抖动较大，可以适当增大该值。如果容易误判已经对齐，应减小该值。

## 串口协议汇总

主机发给视觉端：

```text
ok
multi_anchor
multi_detect
```

视觉端发给主机：

```text
颜色1 颜色2 颜色3 数量1 数量2 数量3
md color dx dy
md none
sx....
```

在 multi 任务解析中，主机会忽略 `sx` 和 `ln` 开头的数据。`md` 数据用于入口对齐和列检测。

## 当前完整执行链路

1. 主机上电，所有舵机复位。
2. 主机进入 `multi_loop()`，电机使能，PS2 可驾驶。
3. 视觉端自动扫描二维码。
4. 视觉端发送二维码原始任务字符串。
5. 主机解析为 `task_queue`，并发送 `ok`。
6. 用户用 PS2 驾驶到色块阵列附近。
7. 用户按 `R3`。
8. 主机将相机转到 `CAMERA_L3_ANGLE_DEG = -77.5 deg` 进入观察状态。
9. 主机发送 `multi_anchor`，视觉端寻找 3x3 左下角色块。
10. 主机根据 `md color dx dy` 闭环调整底盘，直到左下角色块中心对齐画面红框中心。
11. 对齐完成后，当前位置被视为第一列起点。
12. 主机按 `task_queue` 遍历目标颜色。
13. 每个目标从第一列开始，依次检测当前列颜色；不匹配则横移到下一列。
14. 命中目标颜色后，主机再次根据该目标色块的 `dx/dy` 闭环微调，直到色块中心对齐红框中心。
15. 对齐完成后，相机先转到 `CAMERA_INIT_ANGLE_DEG = 0 deg`。
16. 相机到位后，机械臂移动到对应抓取姿态。
17. 夹爪闭合抓取。
18. 机械臂移动到当前放置序号对应姿态。
19. 夹爪打开释放。
20. 底盘转向、机械臂、相机、夹爪复位。
21. 如果还有任务，相机转回 `-77.5 deg` 继续观察和搜索；如果所有任务完成，相机保持 `0 deg`。
22. 返回 `multi_loop()`，PS2 继续可用。

## 调试注意事项

1. 如果 R3 后对齐方向反了，优先检查 `align_multi_left_bottom_anchor()` 中传给 `track_camera_target()` 的符号，以及底盘实际方向。
2. 如果 R3 后一直找不到左下角，检查 `sekuai.py` 的颜色阈值和 `pixels_threshold=1500` 是否过高。
3. 如果入口能对齐但列搜索不准，优先标定 `MULTI_COLUMN_MOVE_MS`。
4. 如果横移一列距离目标为 8 cm，轮半径为 5 cm，且 `MULTI_HORIZONTAL_SPEED_RAD_S = 0.6`，理论 `MULTI_COLUMN_MOVE_MS` 约为 `2667`。
5. 如果抓取行顺序不符合实物堆叠顺序，检查 `grab_pose_index = len(MULTI_GRAB_ROW_POSES) - 1 - row_index`。
6. 如果抓放后下一次动作姿态异常，检查 `reset_multi_action_servos()` 是否被执行，以及各舵机是否实际到位。
7. 如果任务加载偶尔失败，检查主机 UART 日志中二维码任务是否和 `sx` 或 `md` 数据混在一起；当前主机端已经使用短缓冲减少覆盖风险。
