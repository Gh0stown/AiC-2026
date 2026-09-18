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
| HyperLPR3 | 0.1.3 | 中文车牌识别 |
| onnxruntime | 1.19.2 | HyperLPR3 的推理后端 |

自检结果：`torch.cuda.is_available() == False`（本机无显卡）、`rospy` 在 venv 内可用 ✓

## 日常使用

```bash
source .venv/bin/activate
python -c "import torch, ultralytics, cv2; print(torch.__version__)"
```

或直接用 `.venv/bin/python your_script.py`。

## 三个踩过的坑（重建时注意）

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
