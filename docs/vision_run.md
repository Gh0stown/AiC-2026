# 跑视觉识别（比赛方的标准用法：roslaunch + rosrun）

> 三个终端，各司其职。**不搞"一个脚本全包"** —— 只有 `roslaunch` 和 `rosrun`。

```bash
source devel/setup.bash            # 每个终端都要

# ① 世界 + 定位 + 导航（Gazebo/RViz 由它起）
roslaunch competition_robot navigation.launch

# ② 识别节点（会弹出一个窗口显示带 YOLO 框的结果图）
rosrun competition_robot vision_detect.py

# ③ 巡检路线（到识别点位会自动请求识别，并把结果打进本日志）
rosrun competition_robot patrol.py \
    --file $(rospack find competition_robot)/config/recognition_route.yaml
```

## 终端输出

```
[识别] ── A_north（第2站）
         人偶 2 个：社区 2 / 非社区 0        （画面里另有 2 个邻居, 已按组归属排除）
[识别] ── tl_top（第1站）
         红绿灯: 绿灯（直接检灯珠, conf 0.94）
[识别] ── car_1（第14站）
         车牌: 苏A·B8Q62（conf 0.99）

════════════ 识别汇总 ════════════
  街区 A: 共 6 人   社区 6 / 非社区 0
  街区 B: 共 4 人   社区 2 / 非社区 2
  红绿灯: tl_bot=红灯   tl_top=绿灯
  车牌:   car_1=苏A·B8Q62   car_2=鄂D·7B5Q2   car_3=苏A·PL12A
──────────────────────────────────
```

跑完一圈 `patrol.py` 会自动发一次汇总请求，识别节点打印上面这段并**落盘**。

## 每次导航跑的结果独立存档

```
vision_runs/
  2026-09-24_231530/            ← 每次运行一个文件夹（时间戳, 可用 --tag 加后缀）
      images/0001_A_north.jpg   ← 带识别框的结果图（同一张也在窗口里弹出来看）
      images/0002_tl_top.jpg
      summary.txt               ← 终端那份汇总的文本
      results.json              ← 逐点结果 + 汇总（机器可读, 进报告/PPT 直接用）
  2026-09-24_235002/            ← 再跑一次就是新的文件夹, 不会覆盖
```

总文件夹用 `--out-root` 改（默认 `<仓库根>/vision_runs`，已在 `.gitignore` 里）。
**无界面环境**加 `--no-show`（只存图不弹窗）。

## 三个任务的口径（跟模型一致，别搞混）

| 任务 | 模型 | 检的是什么 | 输出 |
|---|---|---|---|
| 人偶立牌 | `weights/standee_yolo11n.pt` | **整块立牌**，2 类（社区 / 非社区人员）| 框 + 类别 → 计数 |
| 红绿灯 | `weights/traffic_light_yolo11n.pt` | ★ **直接检"亮着的那颗灯珠"**（3 类 red/yellow/green_light），**不是灯箱** | 框的类别 = 灯态 |
| 车牌 | `weights/plate_yolo11n.pt` + HyperLPR3 | YOLO 框车牌 → 裁剪（外扩 10 px）→ HyperLPR3 读字符 | 车牌字符串 |

> 灯珠那条特别容易搞错：我们**不做"先检灯箱再判色"**，模型直接输出灯珠类别，
> 所以代码里没有"找灯箱 / 按 15.6%-50%-84.4% 采样"那套逻辑。
>
> 车牌裁剪**别用 0 px**：HyperLPR3 的检测器要背景，但它的**识别网络**偏好紧裁剪；
> 我们用 `plate_ocr.recognize`（纯识别网络），所以留 10 px 左右最稳。

## 为什么不能直接数画面里的框

在 `A_north` 这个点位，画面里往往**同时能看到 A_south 的立牌**（正面 + 邻居的背面都在）。
直接数整幅图的框会把邻居算进去、汇总就重复了。

所以识别节点会用**当前位姿**把"该点位对应的那一组"投影出来，只保留落在该组框里的检测
（`--attribute`，默认开）。拿不到位姿时会退化成"数所有框"并在终端打 ⚠。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/camera/rgb/image_raw` | sensor_msgs/Image | 图像（可用 `--image-topic` 改）|
| 订阅 | `/vision/request` | std_msgs/String | JSON：`{"point":"A_north","kind":"standee","index":2}` |
| 订阅 | `/vision/summary` | std_msgs/String | 空消息 = 打印并落盘汇总 |
| 发布 | `/vision/result` | std_msgs/String | 本次结果 JSON（含框、灯态、车牌、图片路径）|

`patrol.py` 到点后会自动发 `/vision/request`（点位名去掉序号前缀），并等最多
`--vision-timeout`（默认 6 s）把结果打进自己的日志。**不想联动**就给 patrol 加 `--no-vision`。

## 需要权重 + 没权重会怎样

`weights/*.pt` **已经在 git 里**（三个模型都入库了），clone 下来就有；要换模型直接替换同名文件即可。

```
weights/standee_yolo11n.pt   standee_yolo11n.onnx
weights/traffic_light_yolo11n.pt   traffic_light_yolo11n.onnx
weights/plate_yolo11n.pt      plate_yolo11n.onnx
```

缺哪个，启动时会明确打印，**该任务输出"没识别到"但整条流程照跑**（不会崩）。
只训好一部分也能先演示。

## 单独测（不用 ROS）

```bash
python3 tools/vision_infer.py 图片...            # 三类一起跑, 打印结果
python3 tools/vision_infer.py --kind light 图.jpg # 只跑红绿灯
```
