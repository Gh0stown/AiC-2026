# 在另一台机器（主力机 WSL）上从零跑起来

> 本仓库只提交**源码 + 生成的场景资产**（23 MB / 244 个文件）。
> 世界文件、91 个模型（含贴图）、栅格地图、URDF、识别点位/路线配置**都已经在仓库里**，
> 所以 `clone → catkin_make → roslaunch` 就能跑，**不需要官方素材**。
> 官方素材只有"想重新生成场景"时才需要，见 §5。

---

## 0. 前置

* WSL2 + **Ubuntu 20.04**（对应 ROS Noetic；22.04 装不了 Noetic）
* ROS Noetic（推荐 `ros-noetic-desktop-full`，自带 Gazebo 11 / RViz / amcl / move_base / map_server）
* 显存/显卡：**有 GPU 会快很多**。本仓库的沙箱是无 GPU 软件渲染，
  仿真实时率只有 ~0.66，相机一开还掉一半；主力机上应该接近实时。

```bash
# ROS Noetic (若还没装)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list'
sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update && sudo apt install -y ros-noetic-desktop-full
sudo rosdep init 2>/dev/null; rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

本仓库用到的额外包：

```bash
# 必需: 巡航/规划/建图
sudo apt install -y ros-noetic-gmapping ros-noetic-slam-karto \
                    ros-noetic-dwa-local-planner ros-noetic-teb-local-planner
# tools/*.py 依赖 (系统 python)
sudo apt install -y python3-numpy python3-yaml python3-pil \
                    python3-opencv ros-noetic-cv-bridge
# 可选
sudo apt install -y ros-noetic-teleop-twist-keyboard        # 键盘遥控
sudo apt install -y ros-noetic-cartographer-ros             # 复赛加分项 (官方点名)
sudo apt install -y ros-noetic-global-planner ros-noetic-hector-slam ros-noetic-rtabmap-ros
```

---

## 1. clone

```bash
cd ~
git clone git@github.com:Gh0stown/AiC-2026.git          # 需要配 SSH key
# 没有 key 就用 HTTPS:
# git clone https://github.com/Gh0stown/AiC-2026.git
cd AiC-2026
```

---

## 2. 编译 + 冒烟测试

```bash
source /opt/ros/noetic/setup.bash
catkin_make                    # build/ 和 devel/ 没入库, 必须自己编
source devel/setup.bash        # ★ 每个新终端都要 source
```

**★ 建议把下面这行加到 `~/.bashrc`**（本仓库的沙箱里 $HOME 只读加不了，主力机上直接加，
省得每次忘）：

```bash
[ -f ~/AiC-2026/devel/setup.bash ] && source ~/AiC-2026/devel/setup.bash
```

跑起来看看：

```bash
roslaunch competition_robot navigation.launch     # 场地+车+定位+导航+RViz 全套
# 只要看画面:  roslaunch competition_robot robot_gazebo.launch
# 无头(不开界面): roslaunch competition_robot navigation.launch gui:=false rviz:=false
```

RViz 里 `2D Pose Estimate` 点一下车的位置 → `2D Nav Goal` 点目标，车自己过去。

---

## 3. 三条最常用的验收

```bash
./tools/acceptance.sh --quick        # 6 项自动检查, 约 4 分钟 (全量约 12 分钟)
./tools/run_recognition_route.sh     # 跑 17 站巡检路线, 出轨迹图
./tools/run_recognition_route.sh --capture   # 顺带在 10 个识别点各拍 3 帧
python3 tools/show_captures.py captures/     # 把一次拍摄拼成总览图
```

拍照结果在 `captures/<时间戳>/`：`index.csv`（每个点的位姿/文件名/**真值**/红绿灯状态）
+ 图片 + `overview.png`。**`index.csv` 的 `ground_truth` 列可以直接当识别节点的验收答案。**

> ⚠ **端口要对上**：`acceptance.sh` / `run_recognition_route.sh` 内部把
> `ROS_MASTER_URI` 固定成 `http://localhost:11500`（避免和别的 roslaunch 抢）。
> 如果你先手动 `roslaunch`（默认 11311）再跑这两个脚本，它们会连不上你自己的 master。
> 要么都用脚本，要么先 `export ROS_MASTER_URI=http://localhost:11500` 再手动 launch。

---

## 4. 视觉环境（有显卡就装 CUDA 版）

```bash
tools/setup_vision_env.sh --cuda 121     # 显卡机 (CUDA 版本按 `nvidia-smi` 填)
tools/setup_vision_env.sh                # 无显卡则装 CPU 版
.venv/bin/python tools/plate_ocr.py --selftest        # 期望 3/3
.venv/bin/python tools/check_ocr_resolution.py        # 车牌 OCR 分辨率下界
```

`.venv/`、`.cache/` **都没入库**（体积大）。HyperLPR3 的模型目录在
`~/.hyperlpr3`（首次使用会自己下载）；如果想省流量，可以把旧机器的 `~/.hyperlpr3`
整个拷过去，或者放进仓库的 `.cache/home/.hyperlpr3`（代码里有回退逻辑）。

---

## 5. 仓库里**没有**的东西（按需重建）

| 缺什么 | 要不要管 | 怎么弄 |
|---|---|---|
| `build/ devel/` | **必须** | `catkin_make` |
| `.venv/` | 跑视觉才要 | `tools/setup_vision_env.sh --cuda 121` |
| `.cache/` | 自动生成 | 各种脚本自己建；`.cache/home/.hyperlpr3` 可选 |
| `captures/` | 自动生成 | `tools/run_recognition_route.sh --capture` |
| 官方素材 `~/复赛资料/{人员,车辆识别,红绿灯}/` | **只在"重新生成场景"时需要** | 见下 |

场景资产（world / 模型 / 贴图 / 地图）**已入库**，所以正常跑不需要官方素材。
只有要改场地、重做人偶/车牌模型时才需要，对应命令：

```bash
# 素材放到 ~/复赛资料/ 后:
python3 tools/gen_standees.py && python3 tools/setup_standees.py     # 人偶
python3 tools/setup_cars.py                                          # 车辆+车牌
python3 tools/setup_traffic_lights.py                                # 红绿灯
python3 src/competition_arena/scripts/build_arena.py                 # 场地
python3 tools/gen_map_from_arena.py                                  # 栅格地图
python3 tools/gen_recognition_points.py && python3 tools/gen_recognition_route.py
```

---

## 6. WSL 上的注意点

* **界面**：Windows 11 + WSLg 可以直接开 Gazebo/RViz 窗口；老版本 WSL 需要额外 X server，
  或者干脆用 `gui:=false rviz:=false` 无头跑（脚本和验收都支持）。
* **`HOME` 可写**：本仓库的沙箱里 `$HOME` 只读，`acceptance.sh` 会把 `HOME` 重定向到
  `.cache/home`；主力机上不需要，脚本会自动判断。
* **OpenGL**：WSLg 一般能用 D3D12 驱动；报 GL 错时试 `export LIBGL_ALWAYS_SOFTWARE=1`
  （渲染会慢，但功能正常）。
* **文件系统**：仓库**不要放在 `/mnt/c/...`**（跨文件系统 IO 很慢，catkin_make 会慢好几倍），
  放在 WSL 自己的 ext4 里，比如 `~/AiC-2026`。

---

## 7. 到主力机后建议顺手确认的三件事

1. `./tools/acceptance.sh`（**完整跑一遍**）—— 沙箱里 6 项是逐项验证的，
   整条串联没跑完过（沙箱会杀超过 ~10 分钟的任务）。
2. **人眼看一遍视觉质量**：`roslaunch competition_robot robot_gazebo.launch` 之后
   看 Gazebo 里——立牌朝向/贴图、车牌可读、红绿灯三颗透镜（亮的只有一颗）。
   沙箱里没有 GPU，相机画面只能靠抓帧反推。
3. **实车相机姿态**（复赛要求里的 R2）：仿真相机现在是"离地 0.20 m、无俯仰"，
   正对着拍 15 cm 立牌时下沿会切掉一点（可见 91~100%）。
   如果实车相机能加 10° 俯仰或抬高，画面会更好 —— 这条需要你确认实车能不能动。
