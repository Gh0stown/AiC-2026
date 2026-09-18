#!/usr/bin/env python3
"""车牌字符识别 —— 只使用 HyperLPR3 的识别网络，跳过它的检测器。

用法:
    tools/plate_ocr.py 车牌裁剪图.png [更多图片...]
    tools/plate_ocr.py --selftest          # 生成合成车牌并识别, 验证链路

为什么只用识别网络
------------------
实测 (2026-09-18):
  * HyperLPR3 自带检测器 + 纯车牌特写   -> 漏检
  * 自带检测器 + 带背景完整场景         -> 京A12345, 置信度 0.996
  * 识别网络直接喂紧裁剪车牌            -> 京A12345, 置信度 0.996

我们的流程是 YOLO 先框出车牌再裁剪，所以只需要识别网络，
既更快也避开了"需要场景上下文"的限制。

关于模型存放位置
----------------
HyperLPR3 把模型目录硬编码成 $HOME/.hyperlpr3 (见其 config/settings.py)，
**没有环境变量可以覆盖**。若 $HOME 不可写(受限沙箱等)，本模块会自动把
HOME 指到仓库内的 .cache/home (仓库里已缓存好模型)。
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _prepare_home():
    """确保 HyperLPR3 能找到模型目录。必须在 import hyperlpr3 之前调用。"""
    home = os.environ.get("HOME", "")
    local = os.path.join(ROOT, ".cache", "home")
    local_models = os.path.join(local, ".hyperlpr3")
    home_models = os.path.join(home, ".hyperlpr3") if home else ""

    if home and os.path.isdir(home_models):
        return home                      # 正常情况: 模型已在用户家目录
    if os.path.isdir(local_models):
        os.environ["HOME"] = local       # 回退: 用仓库内缓存
        return local
    return home                          # 都没有 -> 交给 hyperlpr3 自己下载


_HOME_USED = _prepare_home()

import cv2                                                            # noqa: E402
import numpy as np                                                    # noqa: E402

try:                                                                  # onnxruntime 会为每个
    import onnxruntime                                                # 权重打印一条 Initializer
    onnxruntime.set_default_logger_severity(3)                        # 警告, 刷屏; 只留 Error
except Exception:
    pass

_RECOGNIZER = None


def _recognizer():
    """惰性加载识别网络 (首次约 1 秒)。"""
    global _RECOGNIZER
    if _RECOGNIZER is None:
        from hyperlpr3.config import settings
        from hyperlpr3.inference.recognition import PPRCNNRecognitionORT

        path = os.path.join(settings._DEFAULT_FOLDER_,
                            settings.onnx_runtime_config["rec_model_path"])
        if not os.path.isfile(path):
            raise RuntimeError(
                "找不到车牌识别模型 %s\n"
                "首次使用需要联网下载 (约 18 MB)。若 $HOME 不可写, "
                "请确认仓库内 .cache/home/.hyperlpr3 存在。" % path)
        _RECOGNIZER = PPRCNNRecognitionORT(path)
    return _RECOGNIZER


def recognize(image_bgr):
    """识别一张**已裁剪好的**车牌图。

    参数: image_bgr —— BGR 图像 (numpy 数组)
    返回: (文本, 置信度); 失败返回 ("", 0.0)
    """
    if image_bgr is None or image_bgr.size == 0:
        return "", 0.0
    if image_bgr.ndim == 3 and image_bgr.shape[2] == 4:
        image_bgr = cv2.cvtColor(image_bgr, cv2.COLOR_BGRA2BGR)
    text, conf = _recognizer()(image_bgr)
    return text, float(conf)


def _synth_plate(text="京A12345", size=(880, 280)):
    """生成一张合成蓝牌, 用于自检。返回 BGR 图像。"""
    from PIL import Image, ImageDraw, ImageFont

    w, h = size
    blue, white = (10, 45, 145), (245, 245, 245)
    img = Image.new("RGB", (w, h), blue)
    d = ImageDraw.Draw(img)
    d.rectangle([3, 3, w - 4, h - 4], outline=white, width=6)

    font_path = None
    for cand in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
                 "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
                 "/usr/share/fonts/truetype/arphic/uming.ttc"):
        if os.path.isfile(cand):
            font_path = cand
            break
    if font_path is None:
        raise RuntimeError("系统缺少中文字体, 无法生成合成车牌")
    f_prov = ImageFont.truetype(font_path, int(h * 0.64))
    f_rest = ImageFont.truetype(font_path, int(h * 0.60))

    # 真实蓝牌版面: 省简称 + 字母, 空出一格(分隔圆点), 再接 5 位字符
    if len(text) >= 3:
        xs = [int(w * 0.035), int(w * 0.235)]
        xs += [int(w * (0.40 + 0.115 * i)) for i in range(len(text) - 2)]
    else:
        xs = [int(w * (0.06 + 0.22 * i)) for i in range(len(text))]

    for i, ch in enumerate(text):
        d.text((xs[i], int(h * 0.16)), ch, font=f_prov if i == 0 else f_rest, fill=white)

    if len(text) >= 3:                                   # 分隔圆点
        cx, r = int(w * 0.375), int(h * 0.035)
        d.ellipse([cx, int(h * 0.76), cx + 2 * r, int(h * 0.76) + 2 * r], fill=white)
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


def _selftest():
    print("HyperLPR3 模型目录:", os.path.join(_HOME_USED or "?", ".hyperlpr3"))
    cases = ["京A12345", "沪B88888", "粤C6D789"]
    ok = 0
    for truth in cases:
        img = _synth_plate(truth)
        text, conf = recognize(img)
        hit = (text == truth)
        ok += hit
        print("  真值 %-10s -> 识别 %-10s 置信度 %.3f  %s"
              % (truth, text or "(空)", conf, "✓" if hit else "✗"))
    print("自检: %d/%d 正确" % (ok, len(cases)))
    return 0 if ok == len(cases) else 1


def main(argv):
    if len(argv) > 1 and argv[1] == "--selftest":
        return _selftest()
    if len(argv) < 2:
        print(__doc__)
        return 2
    rc = 0
    for path in argv[1:]:
        img = cv2.imread(path)
        if img is None:
            print("%-40s 读取失败" % path)
            rc = 1
            continue
        text, conf = recognize(img)
        print("%-40s -> %-12s 置信度 %.3f" % (os.path.basename(path), text or "(未识别)", conf))
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv))
