# 国赛图层的模型来源

## 自建（程序化生成，`tools/gen_national_props.py`）

尺寸依据 = 用户提供的淘宝商品页参考图（国赛道具就是塑料模型，非官方物料）：

| 模型 | 尺寸 | 状态差异 |
|---|---|---|
| `bldg_15` / `bldg_11` / `bldg_20` | 7.2 / 7.8 / 20 cm 高 | `bldg_20` 两扇窗**自发光火焰**（火灾）|
| `bin_red/blue/green/gray` | 4 × 4.5 × 5.5 cm | 红桶**满溢**（盖子掀开 + 垃圾堆）；四色对应四类垃圾 |
| `ebike` / `ebike_toppled` | 18 × 12 cm | 倒伏态整车侧翻 90°；另有违停位姿 |
| `helmet` | 2.9 × 3.2 × 2 cm | — |
| `gauge` | 9 × 3 × 16 cm 立牌 | 指针角度可配（站房仪表那项）|
| `sign` | 7.5 × 1.6 × 26.5 cm | — |
| `heat_bad` / `heat_ok` | 9.5 × 1.8 × 13.5 cm | 异常那块**自发光红** + 高温数值条 |

贴图全部程序化绘制（窗格、分类图标 + 中文类别名、表盘、温度条）。

## 试过但没采用：Gazebo 官方模型库

用代理从 [tjguoyue/gazebo_models](https://github.com/tjguoyue/gazebo_models)
（OSRF `gazebo_models` 的镜像，Apache-2.0）取了 `house_1/2/3`、`apartment`、`dumpster`
（带贴图的正规网格，纹理和造型确实比自建的好看）。

**没采用的原因**：这些 DAE 的网格单位不可靠 —— 试了两种解析（`accessor` 的 min/max、
按 `POSITION` 语义走 `vertices → input → source`）都量不出可信包围盒，按 0.016 缩放后
渲染出来仍是一栋把整个画面塞满的楼（实际 >2 m）。**时间没花在猜单位上**，
留档在 `D:\WSL\gazebo_models\`（原始克隆）与 `D:\WSL\gazebo_models_repo_backup\`
（导入过的那几个），以后要用只要把目录拷回 `models/` 并量准 `<scale>` 即可。
