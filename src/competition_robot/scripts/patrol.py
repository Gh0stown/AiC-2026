#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rosrun 入口: 巡检路线（识别联动已内置）。

    rosrun competition_robot patrol.py --file $(rospack find competition_robot)/config/recognition_route.yaml

★ 实现只有一份, 在 `tools/patrol.py`（那边是唯一真值源, 这里只是让 catkin 能 rosrun）。
  为什么用 importlib 按路径加载: 本文件也叫 patrol.py, 直接 `import patrol` 会导入自己。
"""
import importlib.util
import os
import sys

_WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))          # <仓库根>
_IMPL = os.path.join(_WS, 'tools', 'patrol.py')

spec = importlib.util.spec_from_file_location('patrol_impl', _IMPL)
_mod = importlib.util.module_from_spec(spec)
sys.modules['patrol_impl'] = _mod
spec.loader.exec_module(_mod)

if __name__ == '__main__':
    sys.exit(_mod.main())
