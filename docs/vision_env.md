# 视觉 / 深度学习环境说明

复赛任务一的「视觉识别与检测」（15 分）需要 YOLO + OCR 栈。
本机是 Ubuntu 20.04 + ROS Noetic + Python 3.8，环境情况与常规机器不同，记录如下。

## 本机的限制（所以用了虚拟环境方案）

| 限制 | 现象 | 应对 |
|---|---|---|
| **系统没有 pip** | `python3 -m pip` → `No module named pip`，且 `ensurepip` 缺失 | 从清华镜像手动引导 pip wheel |
| **没有可用 sudo** | `sudo: /usr/bin/sudo 必须属于用户 ID 0` | 全部走 pip，不用 apt |
| **没有 NVIDIA 显卡** | 无 `nvidia-smi`，无 `/usr/local/cuda` | 装 CPU 版 torch；训练搬到显卡机 |
| **`$HOME` 只读** | `touch ~/.local/x` → 只读文件系统 | 一律不写 `$HOME`，缓存放仓库内 `.cache/` |
| **pypi 官方源极慢** | `pypi.org` 约 25 KB/s | 统一走清华镜像（约 1.5 MB/s）|

## 一键重建

```bash
tools/setup_vision_env.sh              # 本机 (CPU 版 torch)
tools/setup_vision_env.sh --cuda 121   # 显卡机 (CUDA 12.1, 版本按实际填)
tools/setup_vision_env.sh --recreate   # 清空重来
```

脚本流程：

1. 在**仓库根**创建 `.venv/`（已 gitignore），不写 `$HOME`、不需要 sudo
2. 若检测到 `/opt/ros` 则带 `--system-site-packages`，让 `rospy` / `cv_bridge` 在 venv 内可见
3. 没有 pip 时从清华镜像拉取兼容 Python 3.8 的 pip wheel 并解包引导
4. 下载并安装 torch / torchvision（CPU 或 CUDA）
5. 装 `ultralytics`、`opencv-python`、`hyperlpr3` 等
6. 预下载 HyperLPR3 模型，最后跑一遍导入自检

## 实测已装版本（2026-09-18）

| 包 | 版本 | 说明 |
|---|---|---|
| PyTorch | **2.4.1+cpu** | Python 3.8 能装的最后一版（2.5 起放弃 3.8）|
| torchvision | 0.19.1+cpu | 与 torch 2.4.1 配对 |
| ultralytics | 8.4.155 | YOLOv8 / YOLO11 |
| OpenCV | 4.10.0 | 固定此版本，更新的 5.x 与 py3.8 生态不匹配 |
| numpy | 1.24.4 | py3.8 的最后一版 |
| matplotlib | **3.7.5** | py3.8 的最后一版；低于 3.3 会让 ultralytics 画图崩，见坑 4 |
| Pillow | **9.5.0** | 满足 ultralytics 的 >=7.1.0，且保留 hyperlpr3 依赖的老 API（10.x 删了）|
| requests | **2.32.3** | 满足 ultralytics 的 >=2.23.0（焦点系统自带 2.22.0 不达标）|
| HyperLPR3 | 0.1.3 | 中文车牌识别 |
| onnxruntime | 1.19.2 | HyperLPR3 的推理后端 |

自检结果：`torch.cuda.is_available() == False`（本机无显卡）、`rospy` 在 venv 内可用 ✓

> 加粗那几行是 2026-09-23 补钉的。原因是 venv 用 `--system-site-packages`，
> 系统自带的老版本会**遮蔽** venv 版本，而 pip 看到"已满足"就不装了 —— 见坑 4。
> `tools/setup_vision_env.sh` 结尾的自检会连**包来自哪个目录**一起打印，
> 以后重建完看一眼就能发现这类问题（`SYSTEM` 且版本不达标的会标 `FAIL`）。

## 日常使用

```bash
source .venv/bin/activate
python -c "import torch, ultralytics, cv2; print(torch.__version__)"
```

或直接用 `.venv/bin/python your_script.py`。

## 四个踩过的坑（重建时注意）

### 1. `download-r2.pytorch.org` 返回 403

PyTorch 索引页里的 href 是绝对 URL，指向 CDN 主机 `download-r2.pytorch.org`，
本机访问该主机 **403 Forbidden**。同路径换到官方主站
`download.pytorch.org` 就正常（实测 1.5 MB/s）。脚本已自动替换主机名。

### 2. `--no-index` 会让依赖也找不到

先试过 `pip install --no-index --find-links .venv/wheels torch`，
结果连 `filelock`、`sympy` 这些依赖都只在本地找，直接失败。
正确做法是 **保留 `--index-url` 走镜像取依赖，只让 torch 从本地 wheel 取**
（因为 `torch==2.4.1+cpu` 这个本地版本号在镜像上不存在，pip 会自动选本地那个）。

### 3. 不加约束会拉下 797 MB 的 CUDA 版 torch

镜像上的 `torch-2.4.1` 是 CUDA 构建，会连带拖 ~2.6 GB 的 `nvidia-*` 依赖。
**必须先装好 torch 再装 ultralytics**，否则 ultralytics 的依赖解析会去拉 CUDA 版。

### 4. 系统老包遮蔽 venv，导致 ultralytics 收尾崩掉（最坑的一个）

venv 是 `--system-site-packages`（为了让 `rospy` / `cv_bridge` 可见），所以
**系统 dist-packages 里的老版本会遮蔽 venv 里的新版本**，而 pip 看到系统版"已满足"
就不再安装。不钉版本的话，下面这几个会悄悄用系统版：

| 包 | 系统焦点版 | ultralytics 要求 | 后果 |
|---|---|---|---|
| **matplotlib** | **3.1.2** | >= 3.3 | **硬崩**：训练跑完、`best.pt` 也存了，但收尾的 `model.val()` / `yolo val` 抛 `AttributeError: 'FontManager' object has no attribute 'addfont'`，拿不到任何指标图 |
| Pillow | 7.0.0 | >= 7.1.0 | 潜伏，可能出怪问题 |
| requests | 2.22.0 | >= 2.23.0 | 潜伏，影响模型下载 |
| numpy | 1.17.4 | >= 1.23 | cv2 报 "compiled against API version 0xe"，pandas 报 `numpy.random has no attribute BitGenerator` |

matplotlib 那条最容易误判：**训练日志看起来全绿**（`100 epochs completed`、
`Optimizer stripped`），只有最后一步 val 崩，很像是"训练有问题"，其实是画图库太老。

诊断看**包来自哪个目录**，光看版本号看不出来：

```bash
.venv/bin/python -c "import matplotlib,PIL,requests,numpy as n; \
  print([(m.__name__, m.__version__, m.__file__) for m in (matplotlib,PIL,requests,n)])"
```

修法就是显式钉版本让 pip 必须装进 venv（`tools/setup_vision_env.sh` 已经这么做了）：

```bash
.venv/bin/python -m pip install --only-binary=:all: \
    --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
    "matplotlib==3.7.5" "Pillow==9.5.0" "requests==2.32.3"
```

Pillow 故意选 **9.5.0** 而不是 10.x：Pillow 10 删掉了 `Image.ANTIALIAS` 这类老 API，
而 hyperlpr3 还在用，升上去有回归风险。实测升到 9.5.0 后
`tools/plate_ocr.py --selftest` 仍是 **3/3**。

`tools/setup_vision_env.sh` 结尾的自检已经改成会打印来源目录并对最低版本做判定，
不达标的直接标 `FAIL`，重建完扫一眼即可。

## 车牌识别（HyperLPR3）实测结论

用 `tools/plate_ocr.py --selftest` 验证（生成合成蓝牌再识别）：

| 方式 | 结果 |
|---|---|
| 自带检测器 + 纯车牌特写 | ❌ 漏检 |
| 自带检测器 + 带背景完整场景 | ✅ 置信度 0.996 |
| **识别网络直接喂紧裁剪车牌** | ✅ 置信度 0.996，自检 **3/3 全对** |

**结论**：它的检测器需要场景上下文，但**识别网络对紧裁剪车牌完美工作**——
正好匹配「YOLO 框车牌 → 裁剪 → OCR」的流程，所以 `tools/plate_ocr.py` 只加载识别网络。

注意 OCR 会**过度自信**：早期排版挤在一起的测试图识别错了，置信度仍有 0.993。
**不要只看置信度，要用真值做校验**。

### 已知失败模式：漏字（中文车牌 OCR 的通病）

最典型的错误是**丢掉一位字符**，尤其是紧跟省简称之后的第 2 位字母
（早期测试里 `粤C6D789` 被识别成 `粤6D789`，置信度仍有 0.993）。
这是这类模型的通病，不是本环境配置问题。

对策按性价比排序：

1. **多帧投票（最有效也最便宜）**——机器人是移动的，同一块车牌在接近过程中
   会被拍到几十帧。对每帧的识别结果做多数表决，能极大压掉偶发漏字。
2. **格式校验**——中国车牌格式固定为 `省简称 + 字母 + 5 位(字母/数字)`，共 7 位。
   识别结果位数不对就直接丢弃并触发重拍，**绝不把可疑结果输出给决策层**。
3. **自训 CRNN**——车牌贴图是我们自己按已知字体生成的，用合成数据训一个专用识别网络，
   在该仿真场景下准确率可以做到接近 100%（能否迁移到真机另说）。

## 两个写死 `$HOME` 的库

- **HyperLPR3**：模型目录硬编码为 `$HOME/.hyperlpr3`，**没有环境变量可覆盖**
  （见其 `config/settings.py`）。`tools/plate_ocr.py` 会自动检测：优先用
  `$HOME/.hyperlpr3`，不可用时回退到仓库内 `.cache/home`
- **ultralytics**：配置写到 `$HOME/.config/Ultralytics`，`$HOME` 不可写时会有警告。
  设 `YOLO_CONFIG_DIR=<仓库>/.cache/ultralytics` 即可

## 和 ROS 的配合

venv 带 `--system-site-packages`，`rospy`、`sensor_msgs`、`cv_bridge` 都能直接 import。
但 venv 里新装的 `numpy` / `cv2` 会**遮蔽**系统版本（系统是 numpy 1.17.4 / cv2 4.2.0）。

- 若 `cv_bridge` 因 ABI 问题报错，可绕开它：ROS Noetic 的 `rgb8`/`bgr8` 编码下，
  图像转 numpy 是纯 Python 几行代码，不需要 `cv_bridge`
- 系统 Python（ROS 节点默认用的）不受影响

## 搬到显卡机

```bash
git clone <repo> && cd <repo>
tools/setup_vision_env.sh --cuda 121      # 按实际 CUDA 版本填: 118 / 121 / 124
```

若显卡机是 **Python 3.10+**，可放开 torch 版本上限：

```bash
TORCH_VER=2.5.1 tools/setup_vision_env.sh --cuda 121
```

`datasets/`、`runs/`、`*.pt`、`*.onnx`、`.cache/`、`.venv/` 都已在 `.gitignore` 里，
搬机器时用网盘/U 盘同步，不要走 git。推理权重放 `weights/`（该目录放行）。

## 待办

- [ ] Gazebo 合成数据集：**官方只给三种状态的参考图，训练数据要自己造**
- [ ] YOLO 训练：红绿灯状态 / 人偶立牌（含类别）/ 车牌框
- [ ] 合成数据自动标注（从 world 文件读真值，不用手标）
- [x] 车牌字符识别链路（`tools/plate_ocr.py`，已用合成牌验证 3/3）

## 数据集与基线（2026-09-23）—— ⚠️ 这一节的标签结论已被推翻

> **先看这条再往下读**：`meta.jsonl` 里的几何框有**系统性竖直偏移**，
> 车牌框整体偏上约一个车牌高，导致"YOLO 框车牌 → 裁剪 → OCR"脚本 **0/37 全错**。
> 详细证据见 `docs/issue_log.md` #10。
>
> 所以下面那些"看起来很好"的指标是**假的**：
> - **mAP 高 ≠ 标签对**。YOLO 是在**学标签**，标签整体偏移它也跟着偏移，
>   plate mAP50 = 0.995 恰恰说明它把错的框学得很准。mAP 只能证明"标签自洽"，
>   不能证明"标签正确"。
> - **edge score 高也是假象**。框整体上移一个车牌高之后，框的**下边界正好压在车牌的
>   上边缘**——那里梯度极强，于是分数很漂亮，但框里装的其实是保险杠。
> - **框的 w/h = 3.04 接近真车牌 3.14 也不能说明问题**：形状对、位置错，
>   比值照样对。
>
> **教训**：验证自动标注，不能只看"指标好不好看"，必须**把框画回图上、并且量一个
> 已知尺寸的目标**。我是靠 `plate_ocr` 端到端 0/37 才回头发现的。

### 已经确认的部分（这些仍然有效）

`tools/gen_vision_dataset.py` 把每个目标的像素框写进了 `datasets/vision/meta.jsonl`
（用 Gazebo 真值位姿做几何投影，带可见度 >=0.6 和小框过滤）。转成 YOLO 目录结构用：

```bash
.venv/bin/python tools/meta_to_yolo.py          # -> datasets/vision_yolo/ (软链接, 不复制图片)
.venv/bin/python tools/meta_to_yolo.py --out datasets/vision_yolo3 --drop-classes traffic_light
```

按**整段 scenario** 留 val（同一段轨迹相邻帧几乎一样，随机抽帧会让 val 虚高）：
train 448 张 / 1494 框，val 152 张 / 460 框，**越界 0、丢弃 0**（这一条是纯几何检查，
不受偏移影响）。

YOLO **检测能力**本身是够的：3 类模型（丢掉 traffic_light）mAP50 = 0.897，
plate 的 P 0.879 / R 1.000。等标签修好之后重新训一遍即可，网络和流程不用改。

标注质量抽查（框边界处的图像梯度 / 全图平均梯度，越大说明框越贴合）。
**别再信这张表了**——框整体上移一个车牌高时，框的下边缘正好落在车牌上边缘，
梯度很强，分数自然漂亮：

| 类别 | n | edge score 中位 | 框 w/h 中位 | 偏移前的解读（已被推翻） |
|---|---|---|---|---|
| standee | 1248 | 4.94 | 0.32 | 立牌是竖长条 |
| non_community | 299 | 6.51 | 0.32 | 同上 |
| plate | 176 | 4.96 | **3.04** | 真车牌 440/140 = 3.14 —— **形状对，位置错** |
| traffic_light | 231 | 2.30 | 3.70 | 最弱 |

基线训练（`yolov8n` / imgsz 640 / batch 16 / 100 epoch，4060 上 7.8 分钟）。
**这些数字只说明"网络能学会这批标签"，不说明标签是对的**，修好标签后要重训：

| 类别 | P | R | mAP50 | mAP50-95 |
|---|---|---|---|---|
| **all** | 0.743 | 0.791 | **0.803** | 0.739 |
| plate | 0.885 | **1.000** | **0.995** | 0.950 |
| standee | 0.843 | 0.912 | 0.938 | 0.856 |
| non_community | 0.613 | 0.827 | 0.839 | 0.811 |
| traffic_light | 0.635 | 0.425 | **0.438** | 0.338 |

3 类模型（`--drop-classes traffic_light`，48 epoch 早停）：mAP50 **0.897**，
plate P 0.879 / R 1.000。

推理 2.4 ms/张 @640（约 400 FPS）。
> 这个模型用的是**偏掉的投影标签**，已经删掉了（`weights/aic_vision_yolov8n.pt` 不存在了）。
> 现在可用的推理权重是 `weights/plate_yolo11n.pt` 和 `weights/traffic_light_yolo11n.pt`。
`weights/` 不入库（`.gitignore` 放行该目录，但 6.2 MB 二进制要不要进 git 由你定）。

### 端到端 `detect_ocr.py`：暴露上面那个偏移的地方

`tools/detect_ocr.py` 把「YOLO 框车牌 → 裁剪 → HyperLPR3 读字符」串起来，
并用 `meta.jsonl` 里的车牌真值打分。结果：

| 环节 | 结果 |
|---|---|
| 车牌**检测** | **37/37 (100%)**，YOLO 置信度 0.93~0.98，框尺寸和标签一致到 1~3 px |
| 车牌**字符识别**（对标签框裁剪） | **0/37** |
| 各种预处理（归一化 / gamma / CLAHE / Otsu / 放大 1~4 倍） | 全是 0/37 |
| 同样输入喂给**合成车牌** | **3/3 正确**（京A12345 @ 0.929） |

也就是说：识别网络没坏、检测也没坏，是**裁剪出来的地方根本不是车牌** ——
框整体偏上，裁到的是保险杠。这就是偏移的实锤，详见 `docs/issue_log.md` #10。

### traffic_light 为什么只有 0.438

**先说明**：修好偏移后这个数字会变，但下面几条结构性问题仍然存在。

不是"目标太小" —— val 里 73 个框**长边全部 > 128 px**，漏检的还都是 279~500 px 的大框。
混淆矩阵显示它 0.45 正确、0.55 被判成背景。原因：

1. **框太宽**：w/h 中位 3.70，框住的是整根横向灯箱（含支架/背板），不是一个灯头。
2. **截断率异常高**：19.9% 的框贴到画面边缘（standee 3.4%、plate 0.0%）。
   `label_of` 会把越界部分 clamp 到画面内，同一目标各帧框的范围不一致。
3. 样本最少：231 框 / 229 帧。

**已定方案：YOLO 不管红绿灯。** `tools/traffic_light.py` 已经 6/6 通过，
红绿灯状态用现有 OpenCV 读颜色就够；YOLO 只负责 standee / non_community / plate。
所以数据集直接用 `--drop-classes traffic_light`。
