#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练视觉检测模型 (YOLO)。

前置: 先准备好一个 YOLO 格式数据集 (images/labels + data.yaml) ——
      现在用 X-AnyLabeling 标注导出。
      (旧的 meta_to_yolo.py「几何投影自动标框」路线已废弃并移出仓库, 见归档分支 archive/sim-dev)

为什么要有这个脚本 (而不直接 `yolo detect train`):
  * `project=` 如果给相对路径, ultralytics 会把它拼到自己的 runs_dir 下面, 于是
    你以为输出在 runs/vision/baseline, 实际在 runs/detect/runs/vision/baseline。
    这里统一算成**绝对路径**。
  * 训练最后一步会自动跑 val 并画图。matplotlib 太老会在这里崩
    (见 docs/issue_log.md #9), 白等一轮。所以在开跑前先体检并给出明确提示。

用法:
    python3 tools/train_vision.py                      # 默认 yolov8n / 640 / 100 epoch
    python3 tools/train_vision.py --model yolov8s.pt --epochs 200
    python3 tools/train_vision.py --device cpu --epochs 5   # 无显卡时快速验证流程
"""
import argparse
import glob
import os
import sys


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default='datasets/vision_yolo/data.yaml')
    ap.add_argument('--model', default='yolov8n.pt',
                    help='预训练权重; 也可指向 weights/ 里已有的 .pt 做增量训练')
    ap.add_argument('--epochs', type=int, default=100)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--device', default='0', help='0 / 0,1 / cpu')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--name', default='baseline')
    ap.add_argument('--patience', type=int, default=30, help='多少轮没提升就早停')
    ap.add_argument('--no-cache', action='store_true', help='关掉内存缓存')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--hsv-h', type=float, default=0.02, help='色调抖动 (0~0.5)')
    ap.add_argument('--hsv-s', type=float, default=0.70, help='饱和度抖动 —— 治明暗/偏色')
    ap.add_argument('--hsv-v', type=float, default=0.45, help='亮度抖动 —— 治"各种光线"(默认比 ultralytics 的 0.4 略强)')
    ap.add_argument('--degrees', type=float, default=5.0, help='旋转角度抖动(度) —— 治拍摄倾角')
    ap.add_argument('--scale', type=float, default=0.50, help='尺度抖动 —— 治远近')
    ap.add_argument('--translate', type=float, default=0.10, help='平移抖动')
    ap.add_argument('--mosaic', type=float, default=1.0, help='mosaic 概率')
    return ap.parse_args()


def preflight():
    """开跑前体检。返回 False 表示有硬问题。"""
    ok = True
    try:
        import torch
        print('torch        %s' % torch.__version__)
        print('CUDA 可用    %s' % torch.cuda.is_available())
        if torch.cuda.is_available():
            print('GPU          %s' % torch.cuda.get_device_name(0))
    except Exception as e:
        print('torch 导入失败: %s' % e)
        return False

    import importlib

    # (import 名, 最低版本, 版本不够会怎样)
    need = [('matplotlib', '3.3.0', '训练收尾画 PR 曲线时会 AttributeError 崩掉'),
            ('PIL', '7.1.0', '图像读写可能出怪问题'),
            ('numpy', '1.23.0', 'cv2/pandas 会报 API 版本不匹配')]

    def ver(s):
        out = []
        for part in s.split('.')[:3]:
            num = ''.join(c for c in part if c.isdigit())
            out.append(int(num or 0))
        while len(out) < 3:
            out.append(0)
        return tuple(out)

    for name, low, why in need:
        try:
            m = importlib.import_module(name)
            v = getattr(m, '__version__', '0')
            f = str(getattr(m, '__file__', ''))
            where = 'venv' if '/.venv/' in f else ('SYSTEM' if 'dist-packages' in f else '?')
            bad = ver(v) < ver(low)
            flag = 'FAIL' if bad else ('warn' if where == 'SYSTEM' else 'ok  ')
            print('%-12s %-10s (%s) %s' % (name, v, where, flag))
            if bad:
                ok = False
                print('             !! 需要 >=%s: %s' % (low, why))
                print('             !! 修法见 docs/issue_log.md #9 或重跑 tools/setup_vision_env.sh')
        except Exception as e:
            print('%-12s 导入失败: %s' % (name, e))
            ok = False
    return ok


def main():
    args = parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(root)

    if not os.path.isfile(args.data):
        sys.exit('找不到 %s —— 先准备 YOLO 数据集: images/labels + data.yaml' % args.data)

    print('================ 体检 ================')
    if not preflight():
        print('\n有硬问题, 先修再跑 (否则多半会在收尾的画图那步崩)。')
        return 1

    from ultralytics import YOLO

    # ★ 绝对路径, 否则会嵌到 runs_dir 下面去
    project = os.path.join(root, 'runs', 'vision')
    print()
    print('================ 训练 ================')
    print('数据 %s' % args.data)
    print('输出 %s/%s' % (project, args.name))
    m = YOLO(args.model)
    m.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
            device=args.device, workers=args.workers, patience=args.patience,
            project=project, name=args.name, exist_ok=True, seed=args.seed,
            cache=not args.no_cache, plots=True, val=True,
            # ★ issue #14: 光照/色调/角度的增强显式写出来, 别只依赖 ultralytics 默认值。
            #   原因: 采集世界能造出的光照差很有限(实测同一位姿只换太阳, 亮度差 1~12%),
            #   而模型要面对的是各种真实光线 —— 这一维只能靠增强补。
            hsv_h=args.hsv_h, hsv_s=args.hsv_s, hsv_v=args.hsv_v,
            degrees=args.degrees, scale=args.scale, translate=args.translate,
            mosaic=args.mosaic)

    run_dir = os.path.join(project, args.name)
    best = os.path.join(run_dir, 'weights', 'best.pt')

    print()
    print('================ 逐类指标 ================')
    r = YOLO(best).val(data=args.data, plots=True, verbose=False)
    print('all              mAP50=%.4f mAP50-95=%.4f P=%.4f R=%.4f'
          % (r.box.map50, r.box.map, r.box.mp, r.box.mr))
    for i, c in enumerate(r.box.ap_class_index):
        print('  %-14s mAP50=%.4f mAP50-95=%.4f' % (r.names[c], r.box.ap50[i], r.box.ap[i]))

    print()
    print('================ 产物 ================')
    for p in sorted(glob.glob(os.path.join(run_dir, 'weights', '*.pt'))):
        print('  %-56s %.2f MB' % (os.path.relpath(p, root), os.path.getsize(p) / 1e6))
    print('  曲线/混淆矩阵: %s/*.png' % os.path.relpath(run_dir, root))
    print()
    print('要把模型交给推理用, 复制到 weights/ (该目录 .gitignore 放行):')
    print('  cp %s weights/vision_yolov8n.pt' % os.path.relpath(best, root))
    return 0


if __name__ == '__main__':
    sys.exit(main())
