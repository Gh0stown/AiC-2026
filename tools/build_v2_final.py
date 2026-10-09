#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v2 采集 → 按 phase 重组 → 只在成功后才删原始目录。

修正上一版的两个错误:
  1) arena 行的任务名映射写成了 'traffic_light', 与 TARGET/KEEP_PHASE 的键不一致 -> KeyError;
  2) 重组和删除写在同一个脚本里, 重组一崩就把原始数据删了 (已发生一次, 数据可复现所以重跑)。
本版: 先全部整理成功 (末尾才删), 并且原始目录放在 v2raw_* 与最终集分开。
"""
from __future__ import annotations
import collections, glob, io, json, os, random, shutil, sys

WS = '/home/ubuntu/AiC-2026'
os.chdir(WS)
random.seed(20261009)

TARGET = {'standee': 340, 'light': 320, 'plate': 220}
KEEP_PHASE = {
    'standee': ('arena', 'ring_div', 'ring'),
    'light': ('light', 'arena'),
    'plate': ('plate', 'arena'),
}
OUT = 'datasets/v2_final'

# 收集所有 v2raw_* 的 meta
rows = collections.defaultdict(list)
for mf in sorted(glob.glob('datasets/v2raw_*/meta.jsonl')):
    d = os.path.dirname(mf)
    for line in io.open(mf, encoding='utf-8'):
        try:
            r = json.loads(line)
        except Exception:
            continue
        ph = r.get('phase')
        if ph == 'arena':
            t = r.get('target', '')
            task = ('light' if t.startswith('tl_') else
                    'plate' if t.startswith('car_') else 'standee')
        elif ph in ('ring', 'ring_div'):
            task = 'standee'
        elif ph in ('light', 'plate'):
            task = ph
        elif ph == 'square':
            continue                       # 密集正方形整批不要
        else:
            continue
        if ph not in KEEP_PHASE.get(task, ()):
            continue
        src = os.path.join(d, r['file'])
        if os.path.isfile(src):
            rows[task].append((src, r, os.path.basename(d)))

if not rows:
    sys.exit('没有可整理的原始数据 (先跑采集)')

# 先建到临时目录, 全部成功再改名, 最后才删原始
tmp = OUT + '.tmp'
shutil.rmtree(tmp, ignore_errors=True)
built = {}
for task, want in TARGET.items():
    items = rows.get(task, [])
    arena = [x for x in items if x[1].get('phase') == 'arena']
    rest = [x for x in items if x[1].get('phase') != 'arena']
    random.shuffle(rest)
    keep = arena + rest[:max(0, want - len(arena))]
    outdir = os.path.join(tmp, task)
    os.makedirs(os.path.join(outdir, 'images'), exist_ok=True)
    mf = io.open(os.path.join(outdir, 'meta.jsonl'), 'w', encoding='utf-8')
    cnt = collections.Counter()
    for i, (src, r, srcdir) in enumerate(keep):
        ext = os.path.splitext(src)[1].lower()
        fn = '%04d_%s_%s%s' % (i, r.get('phase'), r.get('target'), ext)
        shutil.copyfile(src, os.path.join(outdir, 'images', fn))
        mf.write(json.dumps(dict(r, file='images/' + fn, src_dir=srcdir),
                            ensure_ascii=False) + '\n')
        cnt[r.get('phase')] += 1
    mf.close()
    built[task] = len(keep)
    print('  %-8s %3d 张 (候选 %4d)  phase=%s' % (task, len(keep), len(items), dict(cnt)))

# 成功: 落盘 + 删掉原始采集目录
shutil.rmtree(OUT, ignore_errors=True)
os.rename(tmp, OUT)
for d in sorted(glob.glob('datasets/v2raw_*')):
    shutil.rmtree(d, ignore_errors=True)
    print('  已删原始: %s' % d)
print()
for task in TARGET:
    n = len(glob.glob(os.path.join(OUT, task, 'images', '*')))
    print('  %-30s %4d 张' % (os.path.join(OUT, task), n))
