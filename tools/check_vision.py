#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""识别结果核对 —— 从一次巡航的日志 + vision_runs 存档，核对三类识别任务。

为什么需要它
------------
赛题第三条"识别结果输出要求"规定：识别结果须在**终端与图像两处同步呈现、二者一一对应**；
终端要含目标类别与识别内容，**人偶识别需对整个街区进行统计**；图像要对每个目标画框并标注名称与置信度。
而 acceptance.sh 原来只验"场景/灯切换/定位/雷达/巡航/入库/压线"，
**三类识别一个自动检查都没有** —— 这个脚本补上。

用法
----
    python3 tools/check_vision.py --log .cache/acceptance/check5.txt
    python3 tools/check_vision.py --log <巡航日志> --runs vision_runs

核对项（全部只用日志与存档，不需要起仿真）
------------------------------------------
  1 人偶立牌：每个立牌点位的"人偶 N 个"必须等于该点位对应的真实立牌数
  2 街区统计：A/B 街区合计必须等于场上真实数量，且**两类人员都要出现**
  3 车牌：各车牌点读出的字符（归一化）必须等于 config/cars.yaml 的真值
  4 红绿灯：各灯点位必须给出 red/yellow/green（不允许"没看到灯"）
  5 双通道：终端文字与带框图数量一致、逐点有 image 与 boxes（含名称与置信度）

退出码 0=全部通过, 1=有不通过项。
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(WS, 'src', 'competition_arena', 'worlds', 'competition_arena.world')
CARS = os.path.join(WS, 'src', 'competition_arena', 'config', 'cars.yaml')

# ★ 点位从 patrol 的"✓ 01_tl_top 到点"行推 —— `[识别] ── xxx` 那行是识别节点
#   自己的日志, 不在巡航日志里(patrol 只把结果里的 lines 转印出来)。
RE_ARRIVE = re.compile(r'✓\s*\d+_([A-Za-z_0-9]+)\s+到点')
RE_STANDEE = re.compile(r'人偶\s*(\d+)\s*个：社区\s*(\d+)\s*/\s*非社区\s*(\d+)')
RE_PLATE = re.compile(r'车牌:\s*([^\s（(]+)')
RE_LIGHT = re.compile(r'红绿灯:\s*([红黄绿]灯|没看到灯)')
RE_BLOCK = re.compile(r'街区\s*([AB]):\s*共\s*(\d+)\s*人\s*社区\s*(\d+)\s*/\s*非社区\s*(\d+)')
NORM = re.compile(r'[·\-\s.]')


def norm(s):
    return NORM.sub('', (s or '')).upper()


def expected_blocks():
    """从 world 数出每个点位 / 每个街区真实有几个立牌。"""
    txt = open(WORLD, encoding='utf-8').read()
    names = re.findall(r'<name>([AB]_[a-z]+_\d+)</name>', txt)
    per_point, per_block = {}, {}
    for n in names:
        point = n.rsplit('_', 1)[0]
        blk = n[0]
        per_point[point] = per_point.get(point, 0) + 1
        per_block[blk] = per_block.get(blk, 0) + 1
    return per_point, per_block


def plate_truth():
    import yaml
    cfg = yaml.safe_load(open(CARS, encoding='utf-8'))
    out = {}
    for i, c in enumerate(cfg['cars'], 1):
        out['car_%d' % i] = norm(c.get('plate', ''))
    return out


def parse_log(path):
    """-> per_point 结果 dict, 以及街区合计行"""
    got = {}
    blocks = {}
    cur = None
    for ln in open(path, encoding='utf-8', errors='replace'):
        m = RE_ARRIVE.search(ln)
        if m:
            cur = m.group(1)
            got.setdefault(cur, {})
            continue
        if cur:
            m = RE_STANDEE.search(ln)
            if m:
                got[cur]['standee'] = tuple(int(x) for x in m.groups())
            m = RE_PLATE.search(ln)
            if m:
                got[cur]['plate'] = m.group(1)
            m = RE_LIGHT.search(ln)
            if m:
                got[cur]['light'] = m.group(1)
        m = RE_BLOCK.search(ln)
        if m:
            blocks[m.group(1)] = tuple(int(x) for x in m.groups()[1:])
    return got, blocks


def newest_run(runs_dir):
    ds = sorted(glob.glob(os.path.join(runs_dir, '*')), key=os.path.getmtime)
    return ds[-1] if ds else None


def main(argv=None):
    ap = argparse.ArgumentParser(description='识别结果核对（离线，读巡航日志 + 存档）')
    ap.add_argument('--log', default='.cache/acceptance/check5.txt')
    ap.add_argument('--runs', default=os.path.join(WS, 'vision_runs'))
    ap.add_argument('--run', default='',
                    help='指定要核对的存档目录（默认取最新的一个）。'
                         '验收里应显式指向**巡航那一次**的目录 —— '
                         '否则后面为别的检查跑的小任务会新建目录、被误当成巡航结果。')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args(argv)

    if not os.path.isfile(a.log):
        print('  ✗ 找不到巡航日志 %s' % a.log)
        return 1
    got, _ = parse_log(a.log)
    exp_pt, exp_blk = expected_blocks()
    truth = plate_truth()
    fails = []
    sec = {'人偶立牌': [0, 0], '街区统计': [0, 0], '车牌': [0, 0],
           '红绿灯': [0, 0], '双通道': [0, 0]}
    cur_sec = [None]

    def ok(msg):
        print('      ✓ %s' % msg)
        if cur_sec[0]:
            sec[cur_sec[0]][0] += 1
            sec[cur_sec[0]][1] += 1

    def bad(msg):
        print('      ✗ %s' % msg)
        fails.append(msg)
        if cur_sec[0]:
            sec[cur_sec[0]][1] += 1

    # ---- 1 人偶立牌: 逐点位数量 ----
    cur_sec[0] = '人偶立牌'
    print('    1) 人偶立牌（逐点位）')
    for pt, n in sorted(exp_pt.items()):
        if pt not in got or 'standee' not in got[pt]:
            bad('%s 没有识别结果（期望 %d 个）' % (pt, n))
            continue
        gotn = got[pt]['standee'][0]
        (ok if gotn == n else bad)('%s 报 %d 个，真实 %d 个' % (pt, gotn, n))

    # ---- 2 街区统计(结果 JSON 里的 blocks: {A: [社区, 非社区]}) ----
    cur_sec[0] = '街区统计'
    print('    2) 街区统计（赛题要求"对整个街区统计"）')
    run0 = a.run or newest_run(a.runs)
    blocks = {}
    if run0:
        try:
            blocks = json.load(open(os.path.join(run0, 'results.json'),
                                    encoding='utf-8')).get('blocks') or {}
        except Exception:                                   # noqa: BLE001
            blocks = {}
    if not blocks:
        bad('结果里没有街区合计（人偶没按街区统计？）')
    for blk in sorted(exp_blk):
        v = blocks.get(blk)
        if not v:
            bad('街区 %s 没有合计' % blk)
            continue
        com, non = int(v[0]), int(v[1])
        good = (com + non == exp_blk[blk]) and com > 0 and non > 0
        (ok if good else bad)('街区 %s 共 %d 人（社区 %d / 非社区 %d），真实 %d 个，两类都出现=%s'
                              % (blk, com + non, com, non, exp_blk[blk], com > 0 and non > 0))

    # ---- 3 车牌 ----
    cur_sec[0] = '车牌'
    print('    3) 车牌字符')
    for pt, tp in sorted(truth.items()):
        if pt not in got or 'plate' not in got[pt]:
            bad('%s 没有车牌读数（真值 %s）' % (pt, tp))
            continue
        g = got[pt]['plate']
        (ok if norm(g) == tp else bad)('%s 读到 %s（真值 %s）' % (pt, g, tp))

    # ---- 4 红绿灯 ----
    cur_sec[0] = '红绿灯'
    print('    4) 红绿灯状态')
    lights = [p for p in got if p.startswith('tl_')]
    if not lights:
        bad('日志里没有红绿灯识别结果')
    for pt in sorted(lights):
        st = got[pt].get('light')
        (ok if st in ('红灯', '黄灯', '绿灯') else bad)('%s 报 %s' % (pt, st or '无'))

    # ---- 5 双通道 + 存档 ----
    cur_sec[0] = '双通道'
    print('    5) 终端与图像双通道 + 存档')
    run = a.run or newest_run(a.runs)
    if not run:
        bad('找不到 vision_runs 存档目录')
    else:
        imgs = sorted(glob.glob(os.path.join(run, 'images', '*.jpg')))
        rj = os.path.join(run, 'results.json')
        sm = os.path.join(run, 'summary.txt')
        print('        存档: %s' % os.path.relpath(run, WS))
        (ok if os.path.isfile(sm) else bad)('summary.txt 已落盘')
        (ok if os.path.isfile(rj) else bad)('results.json 已落盘')
        n_req = 0
        if os.path.isfile(rj):
            try:
                n_req = len(json.load(open(rj, encoding='utf-8')).get('points', []))
            except Exception:                               # noqa: BLE001
                n_req = 0
        if n_req and len(imgs) == n_req:
            ok('带框结果图 %d 张 == 识别请求 %d 次（终端与图像一一对应）' % (len(imgs), n_req))
        else:
            bad('带框结果图 %d 张 != 识别请求 %d 次' % (len(imgs), n_req))
        if os.path.isfile(rj):
            try:
                d = json.load(open(rj, encoding='utf-8'))
                pts = d.get('points', [])
                withimg = [p for p in pts if p.get('image')]
                withbox = [p for p in pts if p.get('boxes')]
                (ok if len(withimg) == len(pts) else bad)(
                    '逐点都有带框图路径: %d/%d' % (len(withimg), len(pts)))
                (ok if len(withbox) == len(pts) else bad)(
                    '逐点都有框（含名称与置信度）: %d/%d' % (len(withbox), len(pts)))
                b = d.get('blocks') or {}
                s = ' '.join('%s=%d人(社区%d/非社区%d)' % (k, int(v[0]) + int(v[1]), v[0], v[1])
                             for k, v in sorted(b.items())
                             if isinstance(v, list) and len(v) >= 2)
                if s:
                    print('        结果里的街区合计: %s' % s)
            except Exception as e:                          # noqa: BLE001
                bad('results.json 读不了: %s' % e)

    print()
    brief = ', '.join('%s %d/%d' % (k, v[0], v[1]) for k, v in sec.items())
    if fails:
        print('    识别核对: 不通过 %d 项（%s）' % (len(fails), brief))
        return 1
    print('    识别核对: 全部通过（%s）' % brief)
    return 0


if __name__ == '__main__':
    sys.exit(main())
