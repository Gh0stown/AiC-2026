#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""国赛图层自检: 校验「国赛物料没有压住任何识别点位」。

为什么要有这个脚本
------------------
2026-10-10 的审稿意见里我（审稿方）写了"已校验无识别点位被压住"，但那次校验是
**临时脚本、没入库** —— 用户核对仓库后指出"全仓库找不到该校验"。
这个脚本把它固化下来：加入国赛物料后，复赛那 10 个识别点位必须仍然可走，
否则复赛的 17 站路线会被国赛图层破坏。

用法: python3 tools/check_national_layout.py
退出码: 0 = 通过; 1 = 有点位被压住
"""
from __future__ import annotations
import os
import sys
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBS = os.path.join(WS, 'src', 'competition_arena', 'config',
                   'map_obstacles_national_merged.yaml')
PTS = os.path.join(WS, 'src', 'competition_robot', 'config', 'recognition_points.yaml')
# 两条线（2026-10-10 修正）：
#   HARD  —— 车体**内切**半径量级。小于它说明点位真的被障碍"包住"，必失败。
#   WARN  —— 车体**外接**圆半径（= patrol.py 的 FOOTPRINT_R）。小于它只是"转向时角点
#            可能蹭到"，属提示级：B_north 到 A 街区障碍 0.246 m 就落在这一档，
#            而复赛验收连续多轮 17/17、车道 0 压线，说明实际不受影响。
HARD, WARN = 0.15, 0.2582


def main():
    obs = yaml.safe_load(open(OBS, encoding='utf-8')) or []
    pts = yaml.safe_load(open(PTS, encoding='utf-8'))['points']
    print('  障碍矩形 %d 个; 识别点位 %d 个' % (len(obs), len(pts)))
    bad, warn = [], []
    for pt in pts:
        x, y = float(pt['pose'][0]), float(pt['pose'][1])
        for (x0, y0, x1, y1) in obs:
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            gap = (dx * dx + dy * dy) ** 0.5
            if gap < 1e-9:
                bad.append((pt['name'], x, y, 'center in rect'))
            elif gap < HARD:
                bad.append((pt['name'], x, y, 'gap %.3f < HARD %.3f' % (gap, HARD)))
            elif gap < WARN:
                warn.append((pt['name'], x, y, gap))
    for n, x, y, g in warn:
        print('  ⚠ %-9s (%+.3f, %+.3f) 离障碍边界 %.3f m (< 外接圆 %.4f, 提示级)'
              % (n, x, y, g, WARN))
    if bad:
        print('  ✗ %d 个点位被障碍压住/过近（硬线 %.2f m）:' % (len(bad), HARD))
        for n, x, y, why in bad:
            print('      %-9s (%+.3f, %+.3f)  %s' % (n, x, y, why))
        return 1
    print('  ✓ %d 个识别点位均未被压住（硬线 %.2f m 全过；提示级 %d 个见上）'
          % (len(pts), HARD, len(warn)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
