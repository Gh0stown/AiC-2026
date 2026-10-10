# 视觉模型权重与训练证据

> 本目录是**入库的模型权重 + 训练证据**。识别节点按**固定文件名**加载（见
> `src/competition_robot/scripts/vision_detect.py`），所以 clone 下来即可直接跑，
> 不需要自己训练。

## 一、部署的三个模型

三个 `.pt` 与训练 run 的 `weights/best.pt` **逐字节一致**（md5 见下表），
所以报告/文档里引用的指标可以直接回到训练日志核对。

| 任务 | 入库文件 | 来自哪个 run | 训练数据目录 | 类别 |
|---|---|---|---|---|
| 人偶立牌 | `standee_yolo11n.pt` | **`standee_final`**（2026-10-10 换）| `datasets/detect/ring_20260925_102721` | `comm` / `non_comm` |
| 车牌 | `plate_yolo11n.pt` | **`plate_final`** | `datasets/detect/plate_20260923_205130` | `plate` |
| 红绿灯 | `traffic_light_yolo11n.pt` | **`traffic_light_final2`**（2026-10-09 换）| `datasets/detect/light_20261009_224439` | `red_light` / `yellow_light` / `green_light` |

> 训练在 Windows 侧用 X-AnyLabeling 的 ultralytics 训练入口完成（YOLO11n、`imgsz=640`），
> run 目录：`<xanylabeling_data>/trainer/ultralytics/runs/detect/<run>/`。
> 参数逐项见 `train_logs/<run>/args.yaml`。

## 二、指标（口径要对齐，别混用）

| 任务 | run | 训练轮次 | Ultralytics 口径（`results.csv` 末轮 P / R / mAP50 / mAP50-95） | 部署阈值 conf=0.25 下实测 |
|---|---|---|---|---|
| 人偶立牌 | `person_strengthen` | 300（实际 163 停） | **0.990 / 0.990 / 0.990 / 0.913** | 标注数据 **P 92.5% / R 98.7%** |
| 车牌 | `plate_final` | 200（实际 64 停） | **0.997 / 1.000 / 0.995 / 0.820** | 自采 105/105；独立数据 87% |
| 红绿灯 | `traffic_light_final2`（2026-10-09）| 151（早停）| **0.986 / 1.000 / 0.995 / 0.918** | 原始全帧 178 张（有亮灯真值）：**检出 100%、亮灯颜色 98.9%** |
| 红绿灯 | `traffic_light_final`（旧，已换下）| 100（跑满）| 0.987 / 1.000 / 0.995 / 0.882 | 同 178 张：检出/颜色仅 **70.8%** |

> ⚠️ **两个口径不能混着写。** `results.csv` 里的是 Ultralytics 在 best-F1 点上的
> P/R；对外常引用的 92.5% / 98.7% 是**部署阈值**下的评估。报告里两种都写、并标明口径。

## 三、为什么部署 `person_strengthen` 而不是 val 指标更高的 `person_final`

| run | mAP50-95 | 训练集特点 | 比赛场地背板误检 |
|---|---|---|---|
| `person_final`（早期） | **0.994**（更高） | 没有「正面与别人背面同框」的样本 | **41 次** |
| **`person_strengthen`（部署）** | **0.913**（更低） | 补了密集正方形 + 正反面同框硬判据 | **1 次** |

**指标更低、实场更好。** 这正是「mAP 高只能证明标签自洽，不能证明标签正确」的实例：
val 指标奖励的是"把现有标签学准"，而实场失效模式（背板当人）只能靠补数据解决。

## 四、`train_logs/` 里有什么

每个部署 run 一份，用来让文档里的指标可追溯：

| 文件 | 内容 |
|---|---|
| `args.yaml` | **全部训练超参**（模型、数据、轮次、增强、优化器、seed）——文档里的增强参数以它为准 |
| `results.csv` | 逐轮指标（`metrics/precision(B)`、`recall(B)`、`mAP50(B)`、`mAP50-95(B)`）|
| `results.png` | 训练曲线（指标 / 损失 / 学习率）|
| `labels.jpg` | 标注分布图（框中心/尺寸分布，用于判断数据集偏向）|

## 五、还有哪些 run（未入库，属早期或辅助）

| run | 用途 |
|---|---|
| `person_final` | 立牌早期版本（情况太少，已被 `person_strengthen` 取代）|
| `person_light` / `plate_light` | **轻量模型，用于辅助标注**（预标注后人工修）|
| `traffic_light` | 红绿灯早期版本（P 0.82，已被 `traffic_light_final` 取代）|

## 六、`.onnx` 是什么

`*_yolo11n.onnx` 是同一份权重的 ONNX 导出（供不装 torch 的机器推理）。
节点默认用 `.pt`（CPU 推理），换 `.onnx` 见 `docs/vision_run.md`。

## 换型说明（2026-10-09：红绿灯）

外观定稿后（灯罩改用官方 6 张照片材质），旧 `traffic_light_final` 直接失效：
在新外观的 val 上 mAP50 从 0.995 掉到 **0.356**，原始全帧 178 张上"检出+颜色"只有
**70.8%** —— 这就是验收里"灯站通行闸时好时坏"的原因。

新 `traffic_light_final2`（yolo11n，151 轮早停）在**同一批原始全帧**上：
检出 **100%**、亮灯颜色 **98.9%**（绿 56/56、红 49/51、黄 71/71）；
距离 1.2~2.0 m 段颜色 100%，<1.2 m 段 93.5%（太近灯珠出画，属正常）。
旧权重留在 `traffic_light_yolo11n.pt.bak_20260923` 作对照。
训练数据：`datasets/detect/light_20261009_224439`（3 类，train/val 划分）；
日志：`train_logs/traffic_light_final2/`。

## 换型说明（2026-10-10：人偶立牌）

外观定稿（白板 + 贴图层、正面零留白、碰撞体严格 5×15×0.5 cm）后，旧 `person_strengthen`
（2026-09-25）在验收里出现**类别混淆**：B 街区一个社区立牌被误判成非社区
（conf 0.66），导致街区"社区/非社区"报 4/2、真值 5/1。

用户用新数据集 `datasets/detect/person_20261010_193536` 重训 `standee_final`
（yolo11n，80 轮早停）：**P 0.984 / R 1.000 / mAP50 0.995 / mAP50-95 0.985**
（旧模型同口径 0.990 / 0.990 / 0.990 / 0.913）。

> 说明：`datasets/v2_final/standee` 里那 60 张"比赛场地"帧已被用户剪去作训练集，
> 因此无法再做"留出集"独立检验 —— 这类情形的独立检验只能靠**一键验收**（仿真真跑一遍）。
旧权重留在 `standee_yolo11n.pt.bak_20260925` 作对照；日志在 `train_logs/standee_final/`。
