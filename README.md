# AiC-2026 —— 人工智能算法大赛 复赛机器人（ROS Noetic + Gazebo 11）

4.2 m × 4.2 m 比赛场地的**完整仿真与比赛流程实现**：
四轮麦克纳姆全向底盘 → 建图 / AMCL 定位 / move_base 导航 → 17 站固定路线巡检（含 10 个视觉识别点）
→ 三个视觉识别任务（人偶立牌 / 车牌 / 红绿灯）→ 倒车入库。

![场地预览](src/competition_arena/docs/arena_preview.png)

> **第一次跑 / 换机器**：先看 [`docs/wsl_setup.md`](docs/wsl_setup.md)（依赖、clone 后怎么编、WSL 注意事项）。
> **验收**：看 [`验收指南.md`](验收指南.md)（三条命令启动，或 `./tools/acceptance.sh` 一键 6 项检查）。
> **交接 / 现状**：看 [`交接文档.md`](交接文档.md)。
> **一步一坑的排查记录**：看 [`docs/issue_log.md`](docs/issue_log.md)（16 条，含每条的现象 / 根因 / 修法 / 实测）。

---

## 一、实现了什么（都有实测数字）

| 模块 | 内容 | 实测 |
|---|---|---|
| **场地** | 4.2 m 见方；围墙退到白线外 0.10 m、墙高 0.30 m；红绿灯 / 3 辆车 / 人偶立牌按官方尺寸建模 | 地面贴图与三维墙体逐像素对齐（F1 1.000 / 0.932） |
| **机器人** | 四轮麦克纳姆；几何 / 运动学 / 话题名 / frame 名全部来自 `config/robot_params.yaml`（仿真与真机**共用一份**） | 自研全向驱动插件自转保真度 **0.994**（Gazebo 自带插件只有 0.72） |
| **建图** | 默认 2D 雷达（实车同款镭神 N10_P）；gmapping / karto 都能跑并自动打分 | 用几何生成的地图比 SLAM 图可走面积多 0.4 m²、边界更准 |
| **定位** | AMCL（omni 模型，支持麦轮横移） | 定位误差中位 **5.1 cm** |
| **导航** | DWA（默认）/ TEB / TrajectoryPlanner 三套已装并横向对比 | 连续 6 个目标点 **6/6 到达、0 次恢复行为**；直线横向偏差 5 mm |
| **巡检** | 17 站固定路线（10 个识别点，点位与拍照朝向**由几何算出**，不手填） | **17/17 到点、0 失败**；AMCL 到点误差 0.056 m；一圈 166 s |
| **倒车入库** | 激光对墙的位姿伺服倒车（不靠定时 / 定距） | 见 [`验收指南.md`](验收指南.md) 基线值 |
| **视觉·人偶立牌** | `weights/standee_yolo11n.pt`，2 类 `comm` / `non_comm` | 密集正方形数据：精确 **92.5%**、召回 **98.7%**；真实场地 5 个点位计数 **10/10** |
| **视觉·车牌** | `weights/plate_yolo11n.pt` + HyperLPR3 识别网络 | 自采 105 张端到端 OCR **105/105**；独立场地数据 **87%** |
| **视觉·红绿灯** | `weights/traffic_light_yolo11n.pt`，3 类，**直接检"亮着的那颗灯珠"**（不检灯箱） | 自采 90 张 **90/90**；红 / 黄灯原地停车等待，确认绿灯才走 |
| **验收** | `tools/acceptance.sh` 一键 6 项自动检查 | 全绿 |

---

## 二、快速开始

```bash
cd AiC-2026
source /opt/ros/noetic/setup.bash
catkin_make                        # 首次 / 改过 CMakeLists 或新增脚本时
source devel/setup.bash            # 每个新终端都要 source（或直接用 tools/nav.sh）

# ① 世界 + 定位 + 导航
roslaunch competition_robot navigation_wsl.launch      # WSL 用这个；纯 Linux 可用 navigation.launch
# ② 识别节点（会弹出带识别框的结果图；无界面加 --no-show）
rosrun competition_robot vision_detect.py
# ③ 巡检路线（到识别点位自动请求识别）
rosrun competition_robot patrol.py \
    --file $(rospack find competition_robot)/config/recognition_route.yaml
```

只想看场地 / 开键盘开车：

```bash
roslaunch competition_arena arena_only.launch          # 只起场地
roslaunch competition_robot robot_gazebo.launch        # 场地 + 机器人（+ RViz）
rosrun teleop_twist_keyboard teleop_twist_keyboard.py  # 键盘遥控（需 apt install）
```

> **`RLException: ... is neither a launch file in package`？** 那是因为 `devel/setup.bash` 没 source。
> 用包装脚本可以一劳永逸：`tools/nav.sh`（自动 source + 清残留进程 + 避开 `.venv`），
> 支持 `tools/nav.sh planner:=teb` / `sim:=false` / `--build`。

---

## 三、目录结构

```
AiC-2026/                                ← catkin 工作区根
├── README.md                            ← 本文件（项目介绍）
├── 验收指南.md                           ← 怎么启动、怎么判定通过
├── 交接文档.md / PROGRESS.md / 复赛要求与差距分析.md   ← 现状 / 进度 / 需求对照
├── src/
│   ├── competition_arena/               ← 场地功能包
│   │   ├── worlds/competition_arena.world   ★ 比赛场地（地面 + 墙体）
│   │   ├── worlds/collect.world             ★ 采集专用世界（只放相机小车 + 要识别的道具）
│   │   ├── config/                          ★ 真值源：立牌 / 车牌 / 红绿灯尺寸与位姿
│   │   ├── models/                          ← 立牌 / 车 / 红绿灯 / 相机小车模型
│   │   ├── scripts/build_arena.py           ← 从官方平面图重新生成整个场地
│   │   ├── scripts/traffic_light.py         ← 红绿灯切换节点（读 config/traffic_lights.yaml）
│   │   └── docs/                            ← 场地预览 / 坐标网格 / 红绿灯模型说明
│   └── competition_robot/               ← 机器人功能包（四轮麦克纳姆）
│       ├── config/robot_params.yaml         ★ 唯一真值源（仿真 + 真机）
│       ├── config/recognition_points.yaml   识别点位（由 gen_recognition_points.py 生成）
│       ├── config/recognition_route.yaml    最终巡检路线（17 站）
│       ├── config/nav/                      AMCL / move_base / DWA / TEB 参数
│       ├── scripts/                         机器人脚本 + vision_detect.py（识别节点）+ patrol.py（rosrun 入口）
│       ├── src/holonomic_drive_plugin.cpp   自研全向驱动插件
│       └── docs/参数测量清单.md              ★ 拿实车量尺寸看这个
├── tools/                               ← 比赛流程与校验工具（见第七、八节）
├── weights/                             ← 三个识别模型（已入库，clone 即用）
├── maps/arena_clean.pgm/.yaml           ← 导航用的干净地图（几何生成）
├── docs/                                ← 技术文档（视觉环境 / 视觉方案 / 采集世界 / 识别点位 / issue 记录）
└── build/  devel/                       ← catkin 编译产物（不入库）
```

---

## 四、话题与 TF

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | — | **全向**：`linear.x/y` 前后与横移、`angular.z` 自转 |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | 轮式里程计（仿真 / 真机同源） |
| `/odom_groundtruth` | `nav_msgs/Odometry` | 50 Hz | Gazebo 真值，**仅仿真**，用来量定位误差 |
| `/scan` | `sensor_msgs/LaserScan` | 15 Hz | 360° 2D 激光，720 点，0.10–8.0 m |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | 15 Hz | 1280×960 RGB（为车牌 OCR 提的分辨率） |
| `/imu` | `sensor_msgs/Imu` | 100 Hz | |
| `/points` | `sensor_msgs/PointCloud2` | — | 3D 雷达点云，**默认关**（建图 / 导航不用它） |
| `/vision/request` | `std_msgs/String` | — | JSON 请求一次识别（巡检节点发） |
| `/vision/result` | `std_msgs/String` | — | 本次识别结果 JSON（识别节点发） |

TF 树：`map → odom → base_footprint → base_link → {4×wheel, front/rear_caster, laser_link, camera_link, imu_link}`

机器人参数：车体 0.26×0.22×0.09 m、驱动轮 r=45 mm、轮距 0.24 m、底盘离地 20 mm、总重约 2.2 kg。

---

## 五、机器人（四轮麦克纳姆）

几何、运动学、话题名、frame 名**全部来自 `config/robot_params.yaml` 一份参数**，
所以仿真里调好的定位 / 导航参数搬到真车不用改。改参数后重新生成 URDF：

```bash
cd src/competition_robot && python3 scripts/gen_robot.py
```

* **为什么自研驱动插件**：Gazebo 自带的 `gazebo_ros_planar_move` 自转只有指令的约 0.72 倍
  （它的 `SetAngularVel` 会把四个车轮角速度一起锁死）。本包自带 `holonomic_drive_plugin.cpp` 替代，
  自转保真度 0.994。
* **为什么建图用 2D 雷达**：3D 那套插件贴墙 / 在角落时会给出不可能的回波（车头 0.06 m 贴墙却报 3.8~11.5 m），
  gmapping 会在地图外糊出一大片扇形。换 2D 后每一束都和已知几何吻合到 2 cm 内。
* 详细说明与设计取舍见 [`src/competition_robot/README.md`](src/competition_robot/README.md)。

---

## 六、建图 / 定位 / 导航

```bash
roslaunch competition_robot navigation.launch              # 默认：干净地图 + AMCL + move_base(DWA)
roslaunch competition_robot navigation.launch planner:=teb # 目标在中间时更丝滑
roslaunch competition_robot slam_gmapping.launch           # 建图（参数按实测调过）
roslaunch competition_robot slam_karto.launch              # 图优化 SLAM，可对比
python3 tools/gen_map_from_arena.py                        # 从场几何重新生成干净地图
python3 tools/go_to.py 0 0                                 # 不开 RViz 也能发目标点
```

**三个规划器都装好并实测过**（`bash tools/bench_nav.sh <planner>` 可复现）：

| 规划器 | 直线横向偏差 | 贴墙转弯 | 中场转弯 |
|---|---|---|---|
| **dwa（默认）** | 5 mm | ✓ 13.9 s | — |
| teb | 54 mm | ✗ 会摆头 | ✓ 9.2 s（最丝滑，无原地转） |
| traj | 3 mm | ✓ 19.0 s | — |

> 目标点**离墙留 0.4 m 以上**（场地小，膨胀给大了靠墙的点会到不了）。
> 导航默认用**几何生成的干净地图**（`maps/arena_clean.*`），比 SLAM 图边界准、可走面积大。

可选建图方案（本机已装 gmapping / karto / amcl，其余需 `apt install`）：
hector_slam（不需里程计）、cartographer（Noetic 需源码编译）、rtabmap（可做视觉回环）、
LIO-SAM / FAST-LIO（激光惯性 3D，小场地杀鸡用牛刀）、octomap_server（点云转 3D 栅格）。

---

## 七、视觉识别（复赛「视觉识别与检测」15 分）

### 三个模型（已入库，clone 即用）

| 任务 | 权重 | 类别 | 说明 |
|---|---|---|---|
| 人偶立牌 | `weights/standee_yolo11n.pt` | `comm` / `non_comm` | 检**整块立牌**，再按类别计数 |
| 车牌 | `weights/plate_yolo11n.pt` | `plate` | YOLO 框车牌 → 裁剪（外扩 10 px）→ HyperLPR3 **纯识别网络**读字符 |
| 红绿灯 | `weights/traffic_light_yolo11n.pt` | `red` / `yellow` / `green_light` | ★ **直接检"亮着的那颗灯珠"**，框的类别就是灯态；**不做"先检灯箱再判色"** |

推理默认走 **CPU**（`--device cpu`）：三个都是 yolo11n，CPU 上立牌 79 ms / 灯 42 ms / 车牌 113 ms 一张，
够用；而 WSL 的显存和 Windows **共享**，跟 Gazebo 抢会 CUDA OOM，严重时把整个 WSL 带下去（实测踩过）。

### 识别节点与巡检联动（比赛用法：只用 roslaunch / rosrun）

```bash
rosrun competition_robot vision_detect.py            # 识别节点，每轮独立存档
rosrun competition_robot patrol.py \
    --file $(rospack find competition_robot)/config/recognition_route.yaml
```

* 节点订阅 `/camera/rgb/image_raw`；收到 `/vision/request`（JSON）触发一次识别，结果发 `/vision/result`。
* **归属**：画面里常同时出现「本组正面 + 别组背面」。节点按**世界坐标**把检出分给该点位对应的那一组
  （用检出框反算世界坐标，深度由立牌已知高度定），避免把邻居算进来。
* **红绿灯是通行闸**：红 / 黄灯（以及「没看到灯」）原地停车等待，**确认绿灯才放行**；等超时默认停车结束。
  节点在**一次请求内做 3 帧投票**，所以 `--light-confirm` 默认 1，不必再等第二轮。
  灯时长见 `src/competition_arena/config/traffic_lights.yaml`（现为 绿 15 / 黄 3 / 红 10 s）。
* 每次运行存档到 `vision_runs/<时间戳>/`：带框结果图 + `summary.txt` + `results.json`。
* 用法 / 话题表 / **标注约定**见 [`docs/vision_run.md`](docs/vision_run.md)；
  采集世界布局见 [`docs/collect_world.md`](docs/collect_world.md)。

### 识别点位与拍照距离（可直接复算，不手填）

```bash
python3 tools/gen_recognition_points.py     # 约 5 秒，不需要起仿真
```

道具位姿来自 world、尺寸来自各 config、相机模型来自 `robot_params.yaml`、车道线 / 停止线从官方平面图程序提取。
产出 `config/recognition_points.yaml` + `docs/recognition_points.md` + 俯视核对图。

### 最终巡检路线

```bash
python3 tools/gen_recognition_route.py      # 把识别点按弧长插进巡检环线 -> 17 站
./tools/run_recognition_route.sh            # 一键：起仿真 -> 跑整条 -> 存轨迹 -> 重画图
```

路线 = 巡检环线 + 沿途 10 个识别点（离环线都只有 0.01~0.12 m），共 17 站。
每个识别点的动作就是「原地转向 → 直线过去 → 到点原地转到拍照朝向」，正是比赛要求的「到固定点位转向拍照」。
**红绿灯站被排在「同地点其它站之后」** —— 闸必须放在离开路口前的最后一步，否则先判灯再干别的活会闯红灯。

---

## 八、数据集与重训

### 采集世界（只放相机小车 + 要识别的道具）

```bash
python3 tools/build_collect_world.py --shape square --side 0.55     # 立牌摆正方形（正面朝外）
python3 tools/build_collect_world.py --ring-arc 60                  # 或弧形圈
.venv/bin/python tools/gen_collect_dataset.py --rig 1 --out datasets/collect_square \
    --only-phase square --square 240 --div-bucket dense_day         # 采 240 张
```

`gen_collect_dataset.py` **只出图 + `meta.jsonl`**（位姿 / 灯态 / 车牌真值），**框由人在 X-AnyLabeling 里标**。

* 立牌两种摆放：**密集正方形**（边长 0.55 m、正面朝外、相机在外圈拍，每张强制「正面 + 别人背面」同框 ——
  专治「把别的立牌的白色背板认成人」）和**弧形圈多样拍摄**（正对 / 斜视 / 遮挡 / 纯背面四种配方 + 三档相机高度）。
* 红绿灯 / 车牌按**方位角**扫（灯 ±40°/±20°/0°，车牌 ±30°/±15°/0°），因为只采正对时模型在竞技场斜视角会全漏。
* 光照：世界构建器支持 `--sun-diffuse` / `--sun-dir`（分批换太阳）；但实测同一位姿只换太阳亮度只差 1~12%，
  **真正治「各种光线」的是训练增强**（`tools/train_vision.py` 已把 `hsv_h/s/v`、`degrees`、`scale`、`mosaic` 显式化）。

### 重训

```bash
tools/setup_vision_env.sh --cuda 121        # 或本机 CPU 版（见 docs/vision_env.md）
.venv/bin/python tools/train_vision.py --data <你的 data.yaml> --epochs 300
# 训完把 best.pt 覆盖到 weights/ 对应文件名即可（节点按固定文件名加载）
```

---

## 九、验收

```bash
./tools/acceptance.sh          # 一键 6 项：加载 / 话题 / 定位 / 雷达位姿 / 巡航 / 倒车入库
```

逐项标准、预期基线值与手动分步命令见 [`验收指南.md`](验收指南.md)。

---

## 十、已知限制与待办

* **立牌模型**的训练数据以仿真为主（实拍 / 真实光照多样性待补）；背面已作为难负样本覆盖，但**背板本身**
  在极端角度下仍可能被误检。
* **竞技场泛化**：车牌在独立数据上 87%、红绿灯在竞技场视角曾只有 63%（漏检为主）——
  已按方位角补采数据，待重训后复测。
* **红绿灯时长**在仿真里是配置项（`config/traffic_lights.yaml`）；**比赛现场以现场为准**，
  所以节点侧的确认耗时已压到 ~1.8 s 作为保险。
* **投影残余**（`docs/issue_log.md` #10）：按位姿投影的框与画面仍有约 20 px 偏差，
  只影响「用投影做辅助判断」的功能，**不影响识别与计数**（归属已改走世界坐标）。
* **倒车入库**依赖激光对墙，只在墙边 / 角落有效。
* 仓库里保留了**早期一次性脚本**（`tools/bench_*`、`tools/test_mapping*`、`tools/check_*` 等）——
  它们是各阶段问题的排查证据，`docs/issue_log.md` 与 `验收指南.md` 直接引用。

---

## 十一、贡献者

| 角色 | 谁 | 做了什么 |
|---|---|---|
| **项目负责人** | Gh0stown | 赛题解读与关键决策（视觉方案取舍、模型入库、仓库归档策略）、数据标注（密集正方形立牌数据 359 张）、训练（X-AnyLabeling / ultralytics）、真机与实地验收，以及全程把问题反馈回来（"黄灯也得停车等"、"绿灯确认太慢确认完就变黄"、"立牌摆散一点 / 密集一点"、"README 该介绍整个项目"…） |
| **AI 编程助手** | DeepSeek Harness（deepseek-flash） | 跨多次会话承担了绝大部分实现：场地从官方平面图生成与对齐校验、四轮麦克纳姆建模与自研全向驱动插件、AMCL / move_base 调参与三套规划器对比、识别点位与巡检路线生成、三个视觉模型的数据采集管线与识别节点、红绿灯通行闸、数据集与验收工具，以及 `docs/` 下全部技术文档与 16 条 issue 记录 |

**协作方式**：人定「做什么、要什么效果」，AI 负责「怎么实现、跑数据、写文档」。
每个改动都留了可核对的证据 —— `git log` 的 commit message（写清现象 / 根因 / 实测数字）、
`docs/issue_log.md` 的逐条排查记录、以及各脚本的 `--help`。

> 若赛项规定需申报 AI 使用情况，可直接引用本节；不需要就删掉本节，不影响仓库其他内容。

---

## 十二、文档索引

| 文档 | 内容 |
|---|---|
| [`docs/wsl_setup.md`](docs/wsl_setup.md) | 换机器 / 首次运行：依赖、编译、WSL 注意事项 |
| [`验收指南.md`](验收指南.md) | 三条命令启动 + 一键验收 + 6 项判定标准与基线值 |
| [`交接文档.md`](交接文档.md) | 一句话现状 / 接手第一步 / 还差什么 |
| [`复赛要求与差距分析.md`](复赛要求与差距分析.md) | 赛题要求逐条对照 |
| [`docs/issue_log.md`](docs/issue_log.md) | 16 条问题记录：现象 / 根因 / 修法 / 实测数据 |
| [`docs/vision_run.md`](docs/vision_run.md) | 识别节点的用法、话题表、标注约定 |
| [`docs/vision_plan.md`](docs/vision_plan.md) | 视觉任务拆解与方案取舍 |
| [`docs/vision_env.md`](docs/vision_env.md) | 视觉环境（torch / ultralytics / hyperlpr3）与踩过的坑 |
| [`docs/collect_world.md`](docs/collect_world.md) | 采集世界的布局与四个工位 |
| [`docs/recognition_points.md`](docs/recognition_points.md) | 10 个识别点位的位姿、距离与目标像素尺寸 |
| [`docs/arena_build.md`](docs/arena_build.md) | 场地是怎么从官方平面图提取出来的（含 UV 旋转坑、已确认口径） |
| [`src/competition_robot/README.md`](src/competition_robot/README.md) | 机器人包细节与设计取舍 |
| [`src/competition_robot/docs/参数测量清单.md`](src/competition_robot/docs/参数测量清单.md) | 拿实车量尺寸用 |
