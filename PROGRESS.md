# 进度记录 —— 暂停点

> 更新时间：见文件修改时间。下次继续时先看这一页。

---

## 一、当前状态：**全向驱动问题已解决，整套仿真验证通过**

| 部分 | 状态 |
|---|---|
| `competition_arena`（4.2×4.2 场地） | ✅ 完成、验证过 |
| `competition_robot`（麦轮机器人） | ✅ 实车真值参数 + 自研驱动插件，验收通过 |
| 实车参数提取 | ✅ 已从 `~/bit_ws` 读完，已写进 YAML |
| **C++ 全向驱动插件** | ✅ **已完成，自转保真度 0.72 → 0.994** |
| 等你确认的 3 件事 | ⏸ 见第四节 |

最新验收结果（实车真值参数 + 自研插件）：

| 项目 | 结果 |
|---|---|
| 话题 | `/cmd_vel` `/odom` `/odom_groundtruth` `/joint_states` `/imu` `/scan` `/camera/*` 全部 ✅ |
| 线速度保真度 | **1.000** |
| **角速度保真度** | **0.994**（原来 0.72） |
| 里程计 前进/横移/边走边转 | 位置差 **0.2~0.3%** |
| 里程计 自转 | 位置差 1.2%，角度差 0.003 rad |
| 编译 | `catkin_make` 通过（含 C++ 插件），无报错 |

### 本轮解决的核心问题（自转只有 0.72 倍）

**根因**：`gazebo_ros_planar_move` 调 `Model::SetAngularVel()`，
而 Gazebo 这个函数会把模型**及其所有链接**的角速度设成同一个值 ——
把四个车轮**锁死**不能自转，自转时只能刮地，摩擦吃掉角速度。

**验证**：自己写插件时把三种施加方式都实现，同一脚本实测对比：

| 施加方式 | vx | vy | wz |
|---|---|---|---|
| **canonical（只设根 link）** | 1.000 | 1.000 | **0.997** ✅ |
| all_rigid（刚体速度分布） | 0.999 | 1.000 | 0.970 |
| model（= planar_move 的做法） | 1.000 | 1.000 | 0.723 |
| planar_move 原生 | 1.000 | 1.000 | 0.722 |

后两行几乎一致，直接证明根因就是 `SetAngularVel`。**现在默认用 canonical。**

新增文件：
- `src/competition_robot/src/holonomic_drive_plugin.cpp`（约 200 行）
- `tools/test_drive_modes.sh`（四种方式对比测试）

## 一点五、本轮（继续）的进展

### ✅ 深度相机已选定并接通

- **型号：Orbbec Astra Pro** —— 依据是实车 URDF 里 `camera_link` 的 mesh 宽度 **168 mm**，
  和 Astra Pro 实机 **165 mm** 对得上
- 参数（按官方规格）：深度 FOV **58°H × 45.5°V**、测距 **0.6 ~ 8 m**、640×480
- 安装位置沿用实车 URDF：相对 base_link `(0.14914, -0.00049737, 0.19998)`
- 实测：`/camera/rgb/image_raw`、`/camera/depth/image_raw`、`/camera/depth/points`
  全部正常出数据（深度值非零）✅

### ✅ 编码器

仿真里已有完整编码器链路（`sim_wheel_encoders.py` → `/joint_states` →
`mecanum_odometry.py` → `/odom`），里程计实测误差 **0.2~0.3%**。

**实测得到的结论**：仿真里轮子**基本在打滑**（发 vx=0.30 跑 3 秒，
轮子只转 1.56 rad，正常纯滚动应该是 18.6 rad，**只有 8%**）。
所以 Gazebo 自己的关节角度**不能当编码器**，必须由节点模拟。
这也说明：**真机上如果没有编码器、只靠开环积分，里程计会差得很远**——
建议真机一定要把编码器接上。

复现：`bash tools/check_wheel_roll.sh`

### ✅ 雷达：已按你的要求切到 3D

- 16 线 3D 雷达，360° 水平 / ±15° 垂直，10 Hz，`/points` 输出点云
- 用自研的 `scripts/pointcloud_to_scan.py` 从点云切一层生成 **2D `/scan`**
  （系统里没装 `pointcloud_to_laserscan`），给 AMCL / move_base 用
- 实测 `/scan`：360 个 bin（1°/bin），**359 个有回波，只有 1 个没有** ——
  正好就是你说的"后面一小块没覆盖"（车体自遮挡）✅
  中位距离 2.15 m，正好是 4.2 m 场地墙壁的距离

**踩过的三个坑（都修了）：**
1. `gazebo_ros_block_laser` 发的是老的 `sensor_msgs/PointCloud`，不是 `PointCloud2`
   → 转换节点改成用 `rospy.AnyMsg` 两种都收
2. `rospy.AnyMsg` 的 `_type` 永远是 `'*'`，真实类型在 `_connection_header['type']`
3. 实车 URDF 给的雷达高度 0.136 m **落在车体网格内部**（车体 z 范围 0.0275~0.1672），
   仿真里车体是实心盒子，雷达会一直打自己的壳（所有 bin 都有回波且最小距离=range_min）
   → 抬到车顶 z=0.20
4. 转换节点的 bin 宽度（0.5°）比点云分辨率（1°）细，出现"隔一个空一个"的假盲区
   → 改成和点云分辨率对齐

### ✅ 整车质量：按部件估算（没有秤）

`bit_ws` 里没有真实质量数据（只有 CAD 的 0.486 kg 车体，那是纯结构件）。
按部件估得 **3.15 kg** —— 和你估的 3 kg 吻合。

单项最重的是 **16 线 3D 雷达（约 0.9 kg）**，占了近三分之一。
各部件质量都写进 YAML 了，`gen_robot.py` 每次生成时会打印质量账目。
以后有秤了称一下整车，改 `chassis.mass` 对上即可。

### ✅ 用上了实车 STL 模型 + 修了 TF 重复父节点

用户反馈 RViz 里"机器人模型不全、TF 有警告"，两个都修了：

**1. 模型不全** → 之前只有简化几何体（一个盒子 + 圆柱），现在把实车导出的 STL 拷进来用了：
```
meshes/mecanum/{base_link,front_left_wheel,front_right_wheel,back_left_wheel,back_right_wheel}.STL
meshes/sensor/{laser_link,camera_link,imu_link}.STL      合计 7.2 MB
```
- 从 `~/bit_ws/src/robot_description/mowen2/meshes/` 拷的，是**自己车的模型**，不涉及开源模型
- 关键发现：轮子 STL 包围盒是 `0.0968 × 0.0506 × 0.0970`，**Y 方向最薄** → 自转轴就是 mesh 的 Y 轴，
  正好和现在的关节轴一致，所以 mesh **不用加任何旋转**直接用
- 只把 mesh 用作**视觉**，碰撞体仍用简化几何体（STL 做碰撞又慢又不稳）
- 开关：`robot_params.yaml` 里 `simulation.visual_style: mesh | primitive`

**2. TF 警告** → `base_footprint` 有**两个父节点**：
```
odom             -> base_footprint     (mecanum_odometry.py 发的, 要保留)
odom_groundtruth -> base_footprint     (自研插件发的真值, 多余)
```
TF 里一个 frame 只能有一个父节点，两个都发 RViz 会认不出来。
改成插件的真值 TF **默认不发**（`<publishTf>` 开关，默认 false），
真值只通过 `/odom_groundtruth` 话题给。修完 TF 树干净了：

```
base_footprint -> base_link -> {4个轮子}
                            -> {laser_link, camera_link, imu_link}
odom -> base_footprint
```

**3. 顺带发现沙箱的坑**（不影响你）：残留的 gzserver 一直占着 Gazebo 的 **11345 端口**，
导致新起的仿真连到旧的上，测试结果全是错的。测试脚本改成用独立端口了。

### ⚠ 教训：我留下的后台任务会占用端口，导致你的仿真错乱

**现象**（用户报的）：`Spawn service failed`、gazebo 退出码 255、
`TF_REPEATED_DATA`、RViz 里模型一直闪。

**根因**：我调试时启动的一个 GUI 后台任务**一直没关**，占着 ROS 的 **11311** 和
Gazebo 的 **11345** 端口。用户再启动仿真时，新 roslaunch 连到了我那个残留的 master 上，
两套仿真同时往一个 TF 树发数据 → 模型闪、TF 重复、模型名冲突。

**已处理**：杀掉该任务，端口已释放。以后调试完必须确认 `pgrep -a gzserver` 为空。

**排查方法**（以后再遇到"改了不起作用"先查这个）：
```bash
pgrep -a gzserver; pgrep -a gzclient; pgrep -a rosmaster; pgrep -a roslaunch
ss -tln | grep -E '11311|11345'
```

### ✅ 顺带修的两个小问题

1. `sim_wheel_encoders` 加了**仿真时间去重**：时间没推进就不发消息，
   避免同一时间戳重复发布导致 `TF_REPEATED_DATA`。
2. **统一 RViz 和 Gazebo 的外观**：之前我在 URDF 的 `<gazebo>` 里用
   `<material>Gazebo/Orange</material>` 覆盖了颜色，所以 Gazebo 里轮子是橙的、
   RViz 里是 URDF 颜色。现在不覆盖了，两边一致。

### ✅ RViz 里看不到 3D 点云 / 一开就报错 —— 已修

**不是雷达配错了**，`lidar3d.enabled` 一直是 `true`，`/points` 稳定 10 Hz。
用户看到"像 2D"是因为 RViz 里默认高亮的是 `/scan`（我从点云切出来的 2D 扫描）。

真正的问题是 **RViz 显示类用错了**：
```
/points 的实际类型 = sensor_msgs/PointCloud       ← gazebo_ros_block_laser 发的是**老的**类型
我 RViz 里配的     = rviz/PointCloud2             ← 只能订 PointCloud2 → 类型不匹配报错
正确的             = rviz/PointCloud              ← RViz1 有专门订老类型的显示类
```
已改成 `rviz/PointCloud` 并**默认打开**。RViz 配置是运行时加载的，**不用重新编译**，重启即可。

> 顺带记一下: 如果以后想让 `/points` 变成标准的 `PointCloud2`（PCL / octomap 要这个），
> 可以再写个小节点把 PointCloud 转成 PointCloud2 转发一下。

### ✅ 相机 / 出生点 / 建图（本轮）

1. **相机改成只发 RGB**：深度图默认关了（`sensor.publish_depth: false`）。
   用户反馈"深度相机没返回内容"——其实是 Gazebo 出的是 `32FC1` 深度图，
   RViz 的 Image 显示看不了。现在用普通相机插件 `libgazebo_ros_camera.so`，
   只发 `/camera/rgb/image_raw` + `/camera/rgb/camera_info`。

2. **出生点不再放正中间**：默认改成场地西南角 `(-1.50, -1.50)`、朝场地中心（45°）。
   改 `launch/robot_gazebo.launch` 的 arg 或者命令行传 `x:= y:= yaw:=` 都行。

3. **配了建图 launch**：`slam_gmapping.launch` 和 `slam_karto.launch`（都会自动带上仿真）。
   实测确认仿真已提供 SLAM 需要的一切：`/scan`(frame=laser_link, ±180°/1°, 0.2~30m)、
   `/odom`、TF `odom→base_footprint→laser_link`。

   ⚠ **本机 `/opt/ros` 里一个 SLAM 包都没装**，需要用户自己装：
   ```bash
   sudo apt install ros-noetic-gmapping ros-noetic-slam-karto ros-noetic-open-karto \
                    ros-noetic-map-server ros-noetic-amcl ros-noetic-move-base \
                    ros-noetic-teleop-twist-keyboard
   ```
   （apt 源和网络都正常，我验证过 `apt-get install -s` 能解析依赖；
   我沙箱里 sudo 的 setuid 位缺失，所以装不了。）

   `~/bit_ws` 里虽然编译了 slam_karto，但它依赖的 `libsba.so` 没装，也跑不起来。

### ✅ 场地改回只有外沿墙 + 修了里程计初值 bug + 建图验证通过

**1. 场地改回"内部全空"**
`build_arena.py --walls boundary` → 只有 4 面外沿墙，内部是空的。
（之前我误以为要内部隔墙，已改回。）

**2. ★ 修了一个严重的里程计 bug**

出生点从场地中心挪到右上角后，`/odom` 和真值差了 **1.95 m**：

```
真值 (0.484, 1.782, -3.142)   里程计 (1.281, 0.000, 0.000)
```

原因：`mecanum_odometry.py` 的初始位姿**写死成 (0,0,0)**，但车现在出生在
(1.772, 1.782, 180°)。起点偏了 180°，整个里程计就**镜像**了，SLAM 自然建不出正确的图
（表现为地图的墙是弯的、范围比场地大 50%）。

修法：里程计加 `~initial_pose` 参数，launch 把出生位姿传进去。
> 踩坑：`<rosparam>` 的元素文本 **不会**做 `$(arg)` 替换，必须加 `subst_value="true"`。

修完实测：**里程计误差 0.002 ~ 0.007 m**（原来是 1.95 m）。

**3. 建图验证通过**
修完重建，地图从 6.35x7.75 m（变形）变成 **4.70x4.90 m**（场地 4.2x4.2），
墙是直的、free 区域填满内部。

**4. 确认雷达本身没问题**
我一度以为 `/scan` 方向反了，查下来是**我的验证脚本索引错了**——
`/scan` 的 `angle_min = -π`，bin 0 对应的是 **-180°** 不是 0°。
修正后实测和几何完全吻合（0.296/0.297、3.713/3.714、3.819/3.858）。

**遗留**：车**撞墙后**开环里程计还在积分（车实际停了、轮子还在转），
会让地图略微拉长。这是仿真驱动方式的固有现象，不是 bug；
正常跑（不硬顶墙）不会出现。

### ✅ RViz 3D 视图全黑 + "View Controller ... could not be loaded" —— 已修

**现象**：`mapping.rviz` 打开后 3D 面板全黑，弹窗报
`The class required for this view controller, rviz/Orthographic, could not be loaded`，
Views 面板里 `Type` 是空的。

**原因**：和上面 `rviz/PointCloud2` 是同一类错误 —— **我编了一个不存在的类名**。
`rviz/Orthographic` 只是 RViz 界面上的**显示名**，不是插件类名。

本机（rviz 1.14.26）真正注册的 ViewController 只有这 6 个：
```
rviz/FPS   rviz/FrameAligned   rviz/Orbit
rviz/ThirdPersonFollower   rviz/TopDownOrtho   rviz/XYOrbit
```
查法（不用开图形界面）：
```bash
grep -o 'class name="[^"]*"' /opt/ros/noetic/share/rviz/plugin_description.xml | grep -i view
# 或者直接看 base_class_type="rviz::ViewController" 的条目
```

**修**：3 个 `.rviz` 文件里的视图类统一改成 `rviz/Orbit`（俯视：`Pitch: 1.55`）。
`.rviz` 是运行时加载的，**不用重新编译**，重启 RViz 即可。
顺带把 `arena.rviz` 里同样的错也改了，并写了个校验：
把所有 `.rviz` 里的 `Class:` 和 `plugin_description.xml` 里的类名对一遍，
现在 3 个文件全部 OK。

**顺便说明**：`RobotModel` / `Map` / `LaserScan` 这些显示类本来就是对的
（它们在报错前就加载成功了），坏掉的只有 "Current View" 这一个。

### ✅ ★ 地图歪掉的真正原因: 雷达打到了自己车顶 (假墙跟着车走)

**现象**（用户）：键盘往前开，Gazebo 里正常，RViz 里小车"往回抽动"，
走几步地图就整体歪掉。

**先排除掉的两个嫌疑**（都有实测数据，不是猜的）：
1. **里程计没问题**。直行 12 s，`/odom` 和真值 `/odom_groundtruth` 的 x 差
   1~6 mm，twist 也一致；走 5.11 m，编码器里程 5.112 m。
2. **时间配对没问题**。把 scan 按 odom 位姿投到 odom 系，扫了一遍时间偏移
   δ = -0.1~+0.2 s，锐利度差别很小 → 不是时间戳错位。

**真凶**：把 `/scan` 每一束和场地已知几何对一遍，发现**车体局部 147°~213°
的 67 束回波距离恒定为 0.207 m**，而那面墙实际在 0.44 m。

原因：3D 雷达装在车顶上方 0.20 m，它的**下俯光束打到了自己的车顶**；
更近的那些自回波（0.126 / 0.145 / 0.171 m）被 `range_min = 0.2` 滤掉了，
正好剩下 **-9° 那一环打在车顶面上**（0.0327/tan9° = 0.207 m）——
于是 `/scan` 里多出一面**半径 0.21 m、跟着车走的假墙**。

后果（实测对比，直行约 1.7 m）：
```
SLAM 位姿相对里程计:  0.89 m 偏差, yaw 偏 16°   ← 地图歪、变形
map->odom 修正:       0.45 m,  -0.28 rad
```
SLAM 一直想把这面假墙和上一帧画的假墙对齐 = 一直"拽住"车不让它走，
所以轨迹只走了真值的一半，yaw 越跑越偏。

**修法**：在 `pointcloud_to_scan.py` 里加**车体自身回波过滤**：把落在
**车体碰撞盒**内部的点直接丢掉。盒子不是写死的，是拿
`robot_params.yaml` 里的 `chassis` 尺寸 + `sensors.lidar3d.mount` 算出来的，
所以以后挪雷达、改车体，过滤盒自动跟着变（还是"唯一真值源"那套）。

同时顺手发一个 `/points_filtered`（SLAM 真正看到的那层点）给 RViz，
省得被车顶那一圈回波迷惑。RViz 里 `Lidar3D_raw` 保留但默认关掉。

**修完实测**（同一段 5.1 m 直行+转弯）：
```
0.207 m 假回波束数:  67  ->  0
SLAM 相对里程计误差: 0.89 m -> 中位 0.19 m, 最大 0.25 m
map->odom yaw 误差:  16°  ->  -4.9°~+2.4° (末值 -2.7°)
地图:  歪的变形团 -> 384x384 的 4.2x4.2 m 正方房间, 四面墙是直的
```

**再调了 gmapping 参数**（顺手把参数做成 launch arg，方便 A/B）：
```
delta=0.05 sigma=0.05 particles=30 lstep/astep=0.05 iterations=5   -> SLAM 误差中位 0.192 m
delta=0.03 sigma=0.03 particles=60 lstep/astep=0.02 iterations=8   -> SLAM 误差中位 0.059 m  ★现在用这组
```
地图分辨率 5cm -> 3cm，粒子 30 -> 60，搜索步长更细。代价只是建图稍慢一点。

**新增自检工具**：`python3 tools/check_scan_selfhit.py`
（起好仿真后跑一下，会告诉你 `/scan` 里有没有车尾假回波扇形，有就报错退出）。

**遗留（小）**：建出来的图里有一条很细的 "未探索" 缝（斜着一条线），
是机器人路径附近没被扫到的格子，属于正常现象，不影响导航；
真要消掉就多开一段、把那块区域再扫一遍。

**教训**：3D 雷达当 2D 用的时候，"车体自身遮挡"是必查项。
只要雷达装得比车顶高，下俯环就会打到车顶；被 `range_min` 剪掉一部分之后
剩下的那圈会伪装成"一面很近的墙"。验证方法很简单：**把 /scan 和已知几何
对一遍**，看有没有一整段"距离恒定、跟着车走"的回波。

### ✅ ★★ 地图外面那一大片扇形: 3D 点云本身是错的 → 建图改用 2D 雷达

**现象**（用户）：前面一直挺好，某一段之后地图**外面**突然长出一大片扇形，
用户怀疑"墙太矮/车撞墙翻了"。

**先排除"撞墙/翻车"**：让车以 0.5 m/s 死顶墙 40 s，实测真值 `x` 停在 -1.883
（车头正好贴住墙内沿 -2.076，**没有穿墙**），`z` 始终 0（**没翻**）。
所以墙不矮、车也没翻 —— 是数据的问题。

**查下来是 3D 雷达的原始点云不成立**。顶墙时把 `/points` 存下来分析：

```
真值: 车头离墙 0.06 m
点云: 车头那扇区中位距离 = 4.09 m  ← 那是车**背后**那面墙的距离
      有些方向报 11.57 m (场地外面什么都没有) / 99.4 m
      环0(-15°) 中位 0.75 m ✓(地面), 环7 的 z 恒等于 0 (几何上不可能)
```

结论：`gazebo_ros_block_laser` 这个插件在**贴墙 / 进角落**这类位姿下会给出
几何上不可能的回波（穿墙、错位、凭空 11 m）。它本来是给 2D 单线雷达写的。
gmapping 把这些点当墙一画，就是地图外面那片扇形（已稳定复现）。

**修法：建图改用 2D 雷达** —— 顺带更贴近实车（实车就是镭神 N10_P 单线）：

```
lidar2d.enabled: true     # gazebo_ros_laser -> 真正的 sensor_msgs/LaserScan (720 点/圈)
lidar3d.enabled: false    # block_laser 插件不可靠, 默认关 (要看 /points 再打开)
                          # 3D 雷达改用独立 frame lidar3d_link, 不再和 2D 抢 laser_link
scan_from_cloud: false    # 不再用"3D 点云切一层"生成 /scan
```

**同样的死顶墙测试（2D 雷达）**：720 束全部和已知几何吻合，误差 ≤ 2 cm；
比 `range_min`(0.1 m) 还近的方向老实报 `inf`（SLAM 跳过，不再造假墙）。

```
地图: 外面糊一大片的烂图  ->  干净的 4.2x4.2 正方 (空闲 17.0 m², 墙外占用 0 格)
```

**新增工具**：
- `tools/bench_slam.sh <gmapping|karto> [圈数]`：同一套**不撞墙**的小方框路径跑一圈，
  自动存图 + 打分，用来横向比较建图算法。
- `tools/check_map_quality.py <map.pgm>`：地图打分（房间尺寸 / 空闲面积 / 墙直不直 /
  墙外假墙 / 空闲区分几块），并给出 "✓ 正常 / ⚠ 有问题"。
  实测它能准确标出上面那片扇形的烂图（外框 7.89 m、空闲 30.8 m²）。

**教训**：仿真里"地图炸了"先别怀疑算法，先**把传感器原始数据拿出来和已知几何
对一遍**。这次两个坑（车体自回波、插件穿墙回波）都是这么查出来的。

### ✅ 定位 + 导航（AMCL + move_base）已经跑通

新建：`launch/navigation.launch` + `config/nav/*.yaml` + `rviz/nav.rviz` + `tools/go_to.py`。

- **定位**：AMCL，`robot_model_type: omni`（麦轮全向），吃静态地图 + `/scan`(0.1~6m) + `/odom`。
  实测 **AMCL 估计和 Gazebo 真值全程差 2~9 cm**（运动中 2~3 cm）—— 定位这块没问题。
- **导航**：全局 `navfn/NavfnROS`，局部 `base_local_planner/TrajectoryPlannerROS`，
  `holonomic_robot: true`（本机就装了这两个，不用额外 apt）。
- 目标点用 RViz 的 "2D Nav Goal"，或者命令行 `python3 tools/go_to.py x y [yaw]`。

踩到的坑（记下来免得下次再踩）：

1. **膨胀半径别给大**。场地只有 4.2 m，`inflation_radius: 0.35` 时，
   靠墙 0.6 m 以内的目标点（比如 (-1.5,-1.5)）会落进高代价区：move_base
   走到跟前就报 "Rotate recovery behavior started" 原地转圈，永远不宣布到达。
   改成 **0.25** 后正常。**给目标点也别贴着墙**，离墙留 0.4 m 以上。
2. **目标失败时不要马上 cancel**。move_base 正在做恢复行为时把 goal cancel 掉，
   它会卡在"一边转圈一边忽略新目标"的状态，后面所有目标都不动。
   真机比赛里遇到到不了的点，应该先取消再等它停下来（或者直接不取消）。
3. 出生点在角落（离两面墙各 0.29 m），车体 0.38x0.42 m，起步时要先从
   高代价区里挪出来，所以**第一个目标会慢一些**（实测 2.5 m 用了 ~22 s 仿真时间）。

### ✅ 局部规划器换成 TEB（"走不直、原地转圈"的根因与修法）

用户反馈：导航走得不太直；发一条直线目标，快到点才调方向，然后在原地转圈找朝向。

**根因不是"参数没调好"，是本机原来只有最老的一代局部规划器
`base_local_planner/TrajectoryPlannerROS`**：

| 现象 | 原因 |
|---|---|
| 走不直、蛇形 | `holonomic_robot: true` 时它的打分函数在横移方向太平；`sim_time` 只有 1.5 s |
| 快到点才调方向 + 原地转圈 | `latch_xy_goal_tolerance: true`：一进位置容差就停止平移，只原地转到目标朝向 |

**先把它改好**（差速模式 + 关 latch + 容差放宽 + 前瞻 3 s），实测"斜线 3 m + 末端对朝向"：

```
改之前:  57.3 s, 到点前原地转 6.8 s, 蛇形 4 次, 触发 1 次 rotate recovery
改之后:  19.0 s, 到点前原地转 0.8 s, 蛇形 2 次, 0 次恢复行为
直线 3 m: 横向偏差 3 mm, 0 次原地转 (10.9 s 到点)
```

**再上 TEB**（时间弹性带，把整条轨迹当优化问题解 -> 连续曲率、转弯丝滑）：
`sudo apt install ros-noetic-teb-local-planner ros-noetic-global-planner`
（本机没有 sudo，装不了，配置和切换开关都已经写好）

**用户装上 TEB/DWA 之后的实测对比**（同一套 `tools/bench_nav.sh` 测法，指标看
`tools/analyze_nav_run.py`）：

| 规划器 | 直线 3 m 横向偏差 | 直线到点 | 斜线+贴墙目标 (-1.5,-1.5) | 中场转弯目标 (0,-1.0) |
|---|---|---|---|---|
| **dwa（现在的默认）** | **5 mm** | ✓ 9.1 s | ✓ **13.9 s**（端头原地转 1.5 s） | — |
| teb | 54 mm | ✓ 8.9 s | ✗ 端头摆头 33~106 次 → 超时 | ✓ **9.2 s，0 原地转，最丝滑** |
| traj（保底） | 3 mm | ✓ 10.9 s | ✓ 19.0 s（原地转 0.8 s） | — |

**TEB 的两个坑（都是配置问题，已修）**：
1. `max_vel_y: 0.2` + `weight_kinematics_nh: 0` → TEB 会用**横移抄近路**：
   直线横向偏差 76 mm、**69% 的时间在横移**（用户原话"看着太怪"）。
   → 改成 `max_vel_y: 0`（没有横移速度可用）。
2. 矫枉过正把 `weight_kinematics_nh` 设 500 → 它为了满足非完整约束**来回摆头**，
   直线弯 110 mm。→ 给小值 1.0（软约束）。
3. 结论：**TEB 适合"目标在场地中间"**（快、无原地转、连续曲率），
   **靠墙目标用 DWA**（TEB 在墙边会摆头）。
   → `navigation.launch` 默认规划器改成 **dwa**，`teb` 作为可选项。

**顺带验证**：navfn 生成的全局路径是**完美直线**（116 点，到拟合直线最大垂距 0.0000 m），
`map→odom` 在一次 3 m 直行里只漂 0.67° —— 所以"不直"既不是全局规划器的锅也不是定位的锅，
就是局部规划器的跟踪。

**排查用的诊断数据**（值得记）：真值轨迹、编码器里程计、AMCL 轨迹分别对各自最佳拟合
直线求垂距：真值 5.8 cm / 里程计 5.7 cm / AMCL 8.7 cm —— 真值和里程计一致，
说明是**真的在弯**，不是定位画出来的。

新增：
- `config/nav/teb_local_planner_params.yaml`（★ 别开横移、`weight_kinematics_nh` 给小值）
- `config/nav/dwa_local_planner_params.yaml`（按差速车配，走直线稳）
- `navigation.launch` 加 `planner:=teb|dwa|traj` 和 `gplan:=navfn|global`
  （★ 踩坑：参数名不能叫 `global`，那是 python 关键字，`$(eval ...)` 会语法错误）
- `tools/bench_nav.sh` + `tools/analyze_nav_run.py`：同一条路径自动量化
  **横向偏差 / 蛇形指数 / 角速度变化率 RMS / 到点前原地转时长**，用来横向比较规划器

### ✅ 墙角"到不了"的真凶: 局部代价地图把"未知"当障碍（`Cost: -1`）

用户反馈：在墙角处会卡住，迟迟到不了目标点，怀疑是膨胀半径太大。

**先量了地图里的墙厚**（`src/competition_robot/maps/arena.pgm`）：
真实墙体 24 mm，但 **karto 的占用栅格把它糊成了 2 格 = 100 mm**（自由区向内缩 ~60 mm）。
膨胀 0.25 + 车体 inscribed 0.19 一叠加，墙角确实没什么余量了。

**但真凶不是膨胀**。逐级测试（起点场地中心 → 越来越贴墙的墙角目标）：

```
(0.5,0.5) ✓   (1.5,1.5) ✓   (1.7,1.7) ✗ 卡在离目标 5 cm 处永远不结束   (1.82,1.82) ✗
日志: Rotate recovery can't rotate in place ... Cost: -1.00   -> Aborting ...
```

`Cost: -1` = **未知格**。局部代价地图原来 `track_unknown_space: true`，
车自己脚下的格子（激光被自身遮挡；贴墙时前方光束又都是"小于最近量程"的 inf，
清不掉）一直是"未知"；而 `base_local_planner` 把 `NO_INFORMATION` 当碰撞
→ **所有候选轨迹都无效** → 恢复行为也拒绝旋转 → abort。

**修法**：
1. 局部代价地图 `track_unknown_space: false`（未知=可走）← 关键
2. 膨胀 `0.25/4.0` → `0.18/3.0`，`footprint_padding` 0.02 → 0.01
3. 全局规划器 `default_tolerance: 0.30`（目标落进膨胀区时投射到最近可达点）
4. `planner_patience: 3.0`、`controller_patience: 5.0`（真的到不了就快点放弃）
5. `tools/go_to.py` 发目标前用 `/map` 查一次"放得下车吗"，放不下自动挪到最近合法点

**实测**：(1.7,1.7) 从"卡死"变成**正常到达**；`Cost=-1` 6→2 次，恢复行为 12→4 次；
回归：直线 3 m 横向偏差 2.6 cm、零横移零原地转；中场转弯正常。

**结论/规则**：这张图里**目标点离墙内沿至少留 0.30 m**（map 系 `|x|,|y| ≲ 1.75`）；
再往里就是"车体物理上放不下"，谁也没办法，现在会干净 abort 而不是磨蹭。

### ✅ 导航改用"几何生成的干净地图" + 足迹改八边形

用户判断"本质还是地图的问题"——对的。新建 `tools/gen_map_from_arena.py`：
按场地真值(内沿 ±2.076、墙 24 mm)直接生成静态地图, 不经过 SLAM。

| | SLAM 图 | 干净图 |
|---|---|---|
| 分辨率 | 0.05 m | **0.025 m** |
| 墙的内边界 | ±2.015(侵入自由区 60 mm) | **±2.075(真值 2.076, 差 1 mm)** |
| 可走面积 | 16.81 m² | **17.22 m²** |
| 坐标系 | 与世界系差几厘米 | **就是世界系** |
| 直线 3 m 横向偏差 | 2.6 cm | **1.6~2.4 cm** |
| 全局路径直线度 | 0.001 m | 0.001 m |

`navigation.launch` 默认地图已改为 `maps/arena_clean.yaml`。

**足迹从矩形改成八边形**：costmap 的 `inscribed_radius` = 原点到足迹**顶点**的最小距离。
原来的矩形 `[[-0.17,-0.19]...[0.21,0.19]]` 算出来是 **0.255 m**（后两个角其实空着），
八边形贴车体画只有 **0.179 m** → 墙边禁区小了 7.6 cm。
实测墙角可达性: (1.5,1.5) ✓ → **(1.75,1.75) ✓** → (1.82,1.82) ✗。

**墙角极限的真相**（探针读 global costmap 对角线）：
```
1.800 自由 → 1.825 进入膨胀 → 1.900 起"内切禁区"(cost=99)
```
即墙边 0.18 m 是内切带，**再叠加车体前凸 0.20 m** → 圆心要离墙 ~0.38 m。
这是 costmap 的固有设计（内切膨胀 + 多边形足迹各算一次），不是 bug。
`tools/go_to.py` 的可达性阈值同步改成 0.30 m，超了会自动挪点。

**末端"偏出直线"的量化**（干净图, (1.5,0) → (-1.5,0)）：

| 目标朝向 vs 行进方向 | 末端 1 m 横向偏差 | 到点前原地转 |
|---|---|---|
| 一致 | **2.4 cm** | 0 s |
| 差 90° | 8.7 cm（一段弧线） | 1.0 s |
| 差 90° + `latch_xy_goal_tolerance: true` | ~2 cm | 转完再停 |
| 差 90° + `yaw_goal_tolerance: 3.15` | ~6 cm | 0 s |

**结论**：要么"边走边转"(弧线, 默认)，要么"到点再转"(latch)；
两者都要就得用能把转弯规划进轨迹的规划器(TEB, 但本场地会摆头)。
DWA 权重实测: `path_distance_bias`/`goal_distance_bias` 保持 **32/24** 最好
(50/20 → 4.7 cm, 80/10 → 5.4 cm 且变慢); `xy_goal_tolerance` 取 0.12
(0.08 更准但端头会左右微调, 0.15 会提前十几厘米停)。

### ✅ 墙角"卡住"的真正原因: 局部地图的膨胀层画了"内切禁区"

用户截图: 车在墙角停住, 说"其实还能再往前走一点"。探针实测 global costmap 对角线:

```
1.800 自由 → 1.825 进入膨胀 → 1.900 起 cost=99(内切禁区) → 2.050+ 才是墙(100)
```

`costmap_2d` 的膨胀层沿障碍画一圈 `inscribed_radius`(=原点到足迹顶点的最小距离,
八边形 0.179 m)的"内切禁区", **DWA 把 253(内切) 当碰撞** →
车心被挡在 0.18 + 车体前凸 0.20 ≈ **0.38 m** 之外; 目标落进这条带里就反复恢复行为。
(Noetic 的 costmap_2d **不支持**手动设 `inscribed_radius`, 库里没这个参数)

**修法**: **局部代价地图去掉膨胀层**(只留 obstacle_layer)。
实测局部地图代价: 除了墙那格(100), 全是 0 —— 内切禁区没了。
碰撞改由 DWA 的**多边形足迹**判断, 能贴到物理极限; 全局地图仍有膨胀, 路径照样不贴墙。
效果: 墙角测试 **恢复行为 0 次**(之前 12 次), 动作在最近可达点正常结束, 不再卡死。

**另外**: 目标点若超出可达范围, navfn 的 `default_tolerance: 0.30` 会把它投射到
最近的可达格, 于是"到达"其实是在投射点 —— 这就是为什么三个更靠墙的目标都报"到达"
但真值停在 (1.655,1.586)。**结论: 目标点离墙内沿 ≥0.4 m**; 更靠墙的位置用
用户自己的做法(先 `cmd_vel` 转朝向再直线过去)。

### ✅ TEB 最终状态(2026-09-18)

调稳了(圆形足迹 + `dt_ref` 0.25 + 迭代 8/6), 贴墙不再超时, 但:

| | DWA(默认) | TEB |
|---|---|---|
| 直线 3 m 横向偏差 | **2.4 cm** | 3.7 cm |
| 直线到点 | 13.0 s | 12.6 s |
| 贴墙斜线目标 | **✓ 13.9 s, 偏差 0.11 m, 0 恢复** | ✓ 40.9 s, 绕 0.44 m, 1 恢复 |
| 横移 | 0% | 0% |

=> 直线差不多, 靠墙 DWA 明显更好 → **默认保持 DWA**, TEB 用 `planner:=teb` 试。
(定位偏差实测: 干净地图下 AMCL vs 真值 中位 1.5~4 cm)

**用户提的拐角做法**（先 `cmd_vel` 转朝向 → 再直线过去）已记在 §四 待办,
可以做成 `tools/turn_then_go.py`。

### ✅ 航点巡航跑通（图上标点 → 自动识别 → 跑一圈）

用户在我给的网格图上画了 8 个红点（其中 1 个画了圈），要求：出生点 → 第一个拐角 →
后面几个点 → 打圈的点 → 回出生点。做了三个工具：

- `tools/map_pixels.py`：像素 ↔ 场地坐标换算（复用 `build_arena.py` 的 `Frame`），
  并生成 `coord_world_grid.png` / `coord_px_grid.png` 两张标点参考图。
  换算已用出生点验证：world(1.772,1.782) → px(1156,124) = 图右上角 ✓
- `tools/detect_marks.py`：从标注图里**按颜色自动找出红点**并换算成 world 坐标
  （会自动跳过我自己画的红色外框；能认出圆环）
- `tools/patrol.py`：航点巡航 —— **先用 cmd_vel 原地转到方位, 再让 DWA 直着过去**,
  到点后可再原地摆正朝向; 支持 `--loop / --dry-run / --no-return-start / --save-trace`
- `tools/plot_trace.py`：把轨迹画到原图上（`docs/patrol_trace.png`）

**实测**（干净地图 + DWA + `xy_goal_tolerance: 0.07`）：**8/8 航点 + 回起点全部到达**,
一圈 111 s(仿真时间), 到点误差 AMCL 6.1~7.0 cm（真值 3.6~13.7 cm, 差值即定位偏差 ~5 cm）。

**踩过的坑**：`xy_goal_tolerance` 0.12 时每个点都"差 12 cm 就宣布到点"；
因为现在是"先转再走"（末端不会画弧），收紧到 **0.07** 也不会抖 ✓。

**用户反馈后改的两点**：
1. 第一个点不要单独标（原来那个"出生小方块拐角"离出生点 9 cm，车在原地转两次身）
   → `waypoints.yaml` 里删掉，出生点直接是第一站。
2. 回到起点后要**转回出发时的朝向** → `patrol.py` 开跑前记录 yaw，最后用 cmd_vel 转回去
   （实测残余 3.2° → 收紧转向容差后 <1.5°）。想固定角度就在 yaml 写 `start_yaw`。

## 二、这次的重大发现：读到了实车真值

用户把实车工作区放在 `~/bit_ws`，里面有完整的实车代码和模型。

### 2.1 实车机器人模型（`~/bit_ws/src/robot_description/mowen2/`）

SolidWorks 导出的 URDF + STL，**尺寸都是真值**：

| 参数 | 真值 | 我原来猜的 | 差 |
|---|---|---|---|
| 轮半径 | **0.0485 m** | 0.060 | -19% |
| 轮宽 | **0.0506 m** | 0.045 | +12% |
| 前轮 x | **+0.1272 m** | +0.150 | |
| 后轮 x | **−0.0798 m** | −0.150 | |
| 轴距 | **0.2070 m** | 0.300 | **-31%** |
| 轮距 | **0.2908 m** | 0.320 | -9% |
| 车体 (CAD 包围盒) | **0.3334 × 0.2187 × 0.1397 m** | 0.36×0.28×0.14 | |
| 车体中心偏移 | **(+0.0265, 0, +0.0974)** 相对地面原点 | — | |
| 轮子质量 | **0.0515 kg** | 0.40 | **-87%** |
| 车体质量 (CAD) | 0.486 kg | 8.0 | 见下 |

**★ 最重要的两条：**

1. **前后轮 x 不对称**：前轮 +0.1272、后轮 −0.0798，轮子整体前移了 2.4 cm。
   于是自转系数前后不同：**前 0.2726 / 后 0.2252，差 17.4%**。
   如果按对称轴距算，自转指令会偏 10% 左右。
   已经改写了 `mecanum.py` 支持这种布局，并且发现**正运动学里 vy 和 wz 是耦合的**
   （原来的简单公式在不对称时会出错，已改成解 3×3 最小二乘）。

2. **坐标约定**：实车的 `base_footprint` 和 `base_link` **重合，都在地面原点**。
   所以轮心高度 = 轮半径 = 0.0485，传感器安装位置就是"离地高度"。
   我已经把生成器改成同样的约定，现在仿真 URDF 的结构和实车 URDF 一一对应。

### 2.2 实车驱动链路（`~/bit_ws/src/car_bringup/`）

```
/cmd_vel ──→ newt.py ──[串口 0xAA 0xBB 0x0A 0x12 0x02 + x/y/th 各 int16×1000]──→ 下位机
                                                                                    │
/vel_raw ←── pubv.py ←──[串口 12 字节反馈: 0xAA 0xBB + xl xh yl yh zl zh + 校验]────┘
   │
   └──→ base_node (base.cpp) ──→ /odom_raw ──┐
                                             ├──→ ekf_localization ──→ /odom ──→ AMCL + move_base
/wit/imu ────────────────────────────────────┘
```

关键点：
- **真机 `/cmd_vel` 的语义就是 `(vx, vy, wz)`**，和仿真完全一致 ✅
- 反馈里 **`angular.z` 乘了 0.94 的标定系数**（`pubv.py`）
- **真机目前没有读编码器**：`base.cpp` 是把 `/vel_raw` 积分成里程计，属于**开环**
- `base.cpp` 有 `linear_scale_x` / `linear_scale_y` 两个标定系数（launch 里都是 1.0）

### 2.3 ★ 和用户之前说法不一致的地方（**需要确认**）

| 用户说的 | 实车工作区里实际的 |
|---|---|
| 3D 激光雷达（16 线等） | **镭神 N10_P 单线 2D 雷达**（`lslidar_serial.launch`: `lidar_name=N10_P`，`min_range 0.1 / max_range 6`），而且 launch 里**屏蔽了 90°~270°，只用前 180°** |
| 轮式编码器 | 实车**没读编码器**，里程计是开环积分 `/vel_raw` |
| 深度相机 | 工作区里有 `OrbbecSDK_ROS`（femto/deeya/dabai 多款），但**导航 launch 里没启动相机** |

**这三条我没有自己决定，等你确认。**

---

## 三、已经改了的东西（本次）

1. `scripts/mecanum.py` —— 重写
   - 支持前后不对称布局（`front_axle_x` / `rear_axle_x`）
   - 正运动学改成解 3×3 最小二乘（不对称时 vy/wz 耦合）
   - 自检 7 个方向，误差 ~1e-16 PASS

2. `config/robot_params.yaml` —— 全部换成实车真值
   - 每项都标了来源：`[实车]` / `[实测]` / `[待测]` / `[待确认]`
   - 新增 `lidar2d` 段（实车真正装的），`lidar3d` 保留但 `enabled: false`
   - 新增 `encoder.actual_odom_source: open_loop` 记录实车现状

3. `scripts/gen_robot.py` —— 改成实车坐标约定
   - `base_link` 与 `base_footprint` 重合（地面原点）
   - 车体用 `center_offset` 偏移
   - 轮子按 `front_axle_x` / `rear_axle_x` 分别摆放
   - 新增 2D 雷达 SDF（`libgazebo_ros_laser.so`，支持只发前 180°）

---

## 四、下次继续时的待办

### 优先级 0（最新）：导航已经能用了，剩"拐角"这一个体验问题

**现在的默认**：`roslaunch competition_robot navigation.launch`
= DWA 局部规划 + `maps/arena_clean.yaml`（几何生成的干净地图）。
直线走得很直（3 m 横向偏差 2.4 cm），目标点都能到。

```bash
cd ~/桌面/人工智能算法大赛 && source devel/setup.bash
roslaunch competition_robot navigation.launch            # DWA + 干净地图 + RViz
python3 tools/go_to.py 0 0                               # 不开 RViz 也能发目标
bash tools/bench_nav.sh dwa                              # 复现"走多直"的指标
python3 tools/gen_map_from_arena.py                      # 重新生成干净地图
```

**用户提的拐角做法（下次可以做成工具）**：
> "一些拐角用导航不怎么理想，我一般做法是用 cmd_vel 直接转向目标角度，然后就能继续走直线。"

也就是"**先原地转到目标朝向 → 再直线过去**"，绕开规划器在角落里的微操。
可以写成 `tools/turn_then_go.py`：先发 `cmd_vel` 转到目标方向（或 45° 一档一档转），
再调 `/move_base_simple/goal`；比现在的"边走边转"在角落更可控。
（现在的替代开关：`latch_xy_goal_tolerance: true` = 走到点再原地转；
`yaw_goal_tolerance: 1.0~3.15` = 完全不管最终朝向。）

**如果以后还要提升拐角/丝滑**（都验证过效果和数据，见 §一 对应小节）：
1. TEB 全向模式（允许横移）再调一轮 —— 中场目标它 9.2 s 且零原地转，最丝滑，但靠墙会摆头；
2. 给 DWA 做"接近目标 0.5 m 时把 `path_distance_bias` 动态拉高"的小补丁；
3. 手动指定 costmap 的 `inscribed_radius`（0.18 → 0.12）让墙角可达区再往外扩，
   代价是碰撞裕度变小（`go_to.py` 里的 `ROBOT_CLEARANCE` 要同步改）。

### 优先级 0.5：红绿灯 —— 临时版已做出（等物料换材质）

比赛要求 Gazebo 里有能切换的红绿灯（红灯停/绿灯行）。已做完**可行性分析**（未开工）：

**本机实测的能力边界**（探针结果）：
| 能力 | 结论 |
|---|---|
| Gazebo 传输层 `~/visual`（改视觉材质） | 话题存在，但**服务器端没有订阅者**（`gz topic -i` 里 Publishers/Subscribers 都空）→ 只影响 GUI/RViz 显示，**相机传感器看不到** ✗ |
| `gazebo_msgs/Visual.msg` | Noetic 里已删除 ✗ |
| `/gazebo/set_light_properties` / `get_light_properties` / `delete_light` | **有** ✓（改光源颜色/亮度，相机能看到光的变化）|
| `/gazebo/set_model_state` / `spawn_model` / `delete_model` | **有** ✓（纯 ROS 就能挪模型/生成删除）|
| 自研 Gazebo C++ 插件链路 | **已打通**（`holonomic_drive_plugin.cpp` 就是自研 model plugin）✓ |

**三条路线（推荐 A）**：
- **A 物理切换**（推荐，`robustify/gazebo_traffic_light` 就是这个做法）：
  三个灯珠 link + 平移关节，"亮"=推出灯箱窗口、"灭"=缩回箱内 → GUI/相机/雷达都一致 ✓。
  A1 直接用那个插件（BSD-2，43 KB，YAML 时序 + dynamic_reconfigure 强制红/黄/绿/闪；
  注意同一 SDF model 内多个灯共用时序，要独立就把每个灯做成独立 model）；
  A2 自研（复用现有插件工程 ~150 行，状态话题/规则自己定）。
- **B 只用光源 + `set_light_properties`**：零 C++，但灯珠本身颜色不变（弱）。
- **C 只发 `~/visual` 改材质**：**不能用于相机识别**（见上表），只能"看起来在变"。
- 比赛方给的三种灯**图片做材质**可行：本项目地面贴图已经踩通
  `media/materials/{scripts,textures}` + OGRE `.material` 这条路；注意图片点亮样子要给自发光。

**规则实现建议**：灯侧发 `/traffic_light/state`（也便于调试）+ 机器人侧一个交通灯守则节点
（红灯：`/cmd_vel` 置零 + 给 `/move_base/cancel` 发取消；绿灯放行），
`tools/patrol.py` 加 `--respect-traffic-light`。

**待主办方确认**：① 灯怎么切（定时/裁判话题/手动）② 必须视觉识别还是能订阅状态
③ 灯位置/朝向/高度、有无停止线 ④ 三种图片的形式 ⑤ 黄灯规则/红灯能否原地转向 ⑥ 切换周期。

#### 临时版实现（2026-09-18 已做）

用户给了比赛场地示意图（智慧社区赛项介绍 P7），按图做了**两盏临时灯**：

| 文件 | 作用 |
|---|---|
| `config/traffic_lights.yaml` | 唯一真值源：两盏灯的位置/朝向/排布 + 切换时序 |
| `tools/setup_traffic_lights.py` | 生成模型文件 + 把 `<include>` 插进 world（幂等；`--remove` 可撤） |
| `models/traffic_light_v/`、`models/traffic_light_h/` | 竖排/横排两种灯（灯箱+立柱+3透镜+3灯珠） |
| `scripts/traffic_light.py` | 切换节点：时序 + `~/command` 手动覆盖 + `~/state` 状态话题 |
| `launch/traffic_light.launch` | 起切换节点（灯本身随 world 加载） |
| `tools/check_traffic_light.py` | 用机器人相机判断当前是红/黄/绿（识别器雏形） |

位置（世界坐标）：`tl_top (-0.65, 1.88)` 朝 +x（顶部减速带左侧，迎着从起点往左开的车）、
`tl_bot (-0.55, -1.88)` 朝 -x（底部减速带右侧）。灯心高 **0.55 m** 是按相机算的
（相机高 0.20、竖直半视场 ±22.6°、近裁剪 0.6 → 距离 ≥0.84 m 能识别，所以停车线放 1.2~1.5 m）。

**实测验证到的**：
- 两盏灯随 world 加载 ✓（`get_world_properties` 里有 `tl_top`/`tl_bot`）
- 切换机制 ✓：`tl_top::j_red` 关节 `+0.05`(亮) ↔ `-0.03`(缩进灯箱)，灯珠 link 跟着移动 ✓
- `/traffic_light/state` / `/traffic_light/command` ✓
- 相机**能看到灯** ✓（`docs/traffic_light_camera.png` 就是机器人相机拍的：黑灯箱+亮着的红灯）

**Gazebo 踩坑（都实测过，很重要）**：
1. 发 Gazebo 的 `~/visual` 改材质：**无 GUI 时该话题没有任何订阅者**（`gz topic -i` 空），
   只影响 GUI/RViz，**服务器端相机看不到** → 做视觉识别不能用这条路
2. `Noetic` 的 `gazebo_msgs` 里**没有 `Visual.msg`**（老 ROS 改 visual 的话题已删）
3. `<static>true</static>` 的模型用 `SetModelState` 挪：位姿变了但**渲染不跟随**
4. `<kinematic>true</kinematic>`：本机 sdformat **不认**，灯珠直接掉地上
5. 运行中动态 `spawn_sdf_model` 生成模型：**服务器端相机渲染会冻住**（相机还在发消息，
   画面永远不变）→ 所以灯必须**写进 world**，运行时只切关节 ✓
6. 结论：**只有"关节运动"是 Gazebo 一定会正确渲染的机制**（那个开源插件用的也是 `joint->SetPosition`）

**待确认的一件事**：我的沙箱没有 GPU（软件渲染），相机画面在状态切换时没看到变化，
所以"相机能否实时看到红/绿灯切换"要**在你的机器上确认**（一条命令）：
```bash
# 车开到灯前 1.2~1.5 m, 然后
rostopic pub -1 /traffic_light/command std_msgs/String "data: 'red'"
python3 tools/check_traffic_light.py --save /tmp/red.png
rostopic pub -1 /traffic_light/command std_msgs/String "data: 'green'"
python3 tools/check_traffic_light.py --save /tmp/green.png
```


### 优先级 1：把当前状态验完
```bash
cd ~/桌面/人工智能算法大赛
cat .progress_test.txt          # 看这次冒烟测试结果
source devel/setup.bash
roslaunch competition_robot robot_gazebo.launch
```

### 优先级 2：等用户确认三件事
1. 雷达到底是 **2D N10_P** 还是真的要换 **3D 16 线**？
2. 下位机**有没有编码器**回传？现在里程计是开环的。
3. 深度相机型号（Orbbec 哪一款）？导航里要不要用？

还需要用户**上秤称整车质量**（现在填的 1.20 kg 是估的，CAD 只有 0.486 kg）。

### 优先级 3：~~写 C++ 全向驱动插件~~ ✅ **已完成**
见上。自转保真度 0.72 → 0.994。
如果以后想换 Gazebo 版本，用 `tools/test_drive_modes.sh` 重测一遍，
必要时改 `config/robot_params.yaml` 里的 `simulation.velocity_mode`。

### 优先级 4：把真机驱动也做成 sim2ros 的一部分
现在真机用的是 `newt.py`/`pubv.py`/`base.cpp`（串口协议 + 开环里程计）。
可以改成读**同一份 YAML**，用**同一个 `mecanum.py`**，
这样仿真里标定的东西真机直接能用。

---

## 五、文件位置速查

```
~/桌面/人工智能算法大赛/           ← 我的仿真工作区
├── PROGRESS.md                    ← 本文件
├── README.md                      ← 总说明
├── src/competition_arena/         ← 4.2×4.2 场地（已完成）
├── src/competition_robot/
│   ├── config/robot_params.yaml   ← ★ 唯一真值源（已填实车值）
│   ├── scripts/mecanum.py         ← ★ 运动学（仿真/真机共用）
│   ├── scripts/mecanum_odometry.py← ★ 里程计（仿真/真机共用）
│   ├── scripts/gen_robot.py       ← YAML → URDF
│   ├── docs/参数测量清单.md        ← 测量清单（部分已被实车数据取代）
│   └── README.md                  ← 设计取舍与已知限制
└── tools/
    ├── test_robot.sh              ← 一键验收
    └── verify_alignment.sh        ← 校验场地贴图对齐

~/bit_ws/                          ← 用户提供的实车工作区（只读参考）
├── src/robot_description/mowen2/  ← ★ 实车完整 URDF + STL（真值来源）
├── src/car_bringup/               ← ★ 底盘驱动（base.cpp / newt.py / pubv.py）
└── src/{leishen,OrbbecSDK_ROS,wit}/ ← 雷达 / 相机 / IMU
```
