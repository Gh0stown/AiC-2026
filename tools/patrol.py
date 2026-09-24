#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航点巡航 —— 按用户的做法: **先用 cmd_vel 原地转到方位, 再让 move_base(DWA) 走直线**。

为什么这么做: DWA 在"边转边走"时末端会画一段弧线(实测 90° 转向偏 8~9 cm),
而在原地把朝向先对好、再直线过去就非常准(实测 3 m 直线横向偏差 2.4 cm)。

用法:
    python3 tools/patrol.py                          # 用 config/waypoints.yaml
    python3 tools/patrol.py --file xxx.yaml
    python3 tools/patrol.py --dry-run                # 只打印路线 + 各点旋转余量
    python3 tools/patrol.py --no-return-start        # 最后不回起点
    python3 tools/patrol.py --loop                   # 一直循环
    python3 tools/patrol.py --save-trace /tmp/t.csv  # 记录轨迹(画图用)
    python3 tools/patrol.py --goal-includes-yaw      # 旧行为, 见下
    python3 tools/patrol.py --no-park                # 不做倒车入库

倒车入库 (总决赛"泊车入位 + 朝向"项):
    航点跑完后, 先用 move_base 开到配置里的"起倒点", 原地转到背离库位的朝向,
    再用 pose_servo() 倒进去。**不写死距离也不定时** —— 终止条件是"位姿到位",
    控制器每帧取当前位姿, 用 cmd_vel 闭环收敛, 倒多远是结果不是输入。
    位姿优先由"激光对墙"给出(拟合最近两面墙, 不依赖 AMCL/里程计, 角落处厘米以内),
    拿不到才退回 AMCL。配置见 waypoints.yaml 的 reverse_park。
    不想倒车: --no-park。

朝向是怎么处理的 (重要):
    * 出发前用 cmd_vel 原地转到"行进方位" (--no-pre-rotate 可关)
    * move_base 的**目标朝向 = 行进方向**, 也就是到点几乎不需要原地转
    * 到点后若要某个朝向(航点的 yaw / 回程的 start_yaw), 再用 cmd_vel 原地摆正
    => 所有原地转向都走 cmd_vel, **完全不经过规划器**。这样即使终点贴着墙,
       也不会因为"转向余量 < 定位误差"被判成碰撞而偶发卡住, 180 度掉头也稳定。
       旧行为(把最终朝向下给 move_base)可用 --goal-includes-yaw 恢复。

航点文件格式 (world 和 pixel 二选一, pixel 按 coord_world_grid.png 那张图的像素):
    start: [1.772, 1.782]        # 出生点 (= navigation.launch 里的 spawn)
    return_to: [1.672, 1.682]    # 可选: 最后回到哪 (不写就用 start)
    waypoints:
      - {name: A, pixel: [1138, 145]}          # 或 world: [1.709, 1.709]
      - {name: B, world: [0.0, -1.0], yaw: 1.57}   # 可选: 到点后再原地转到这个朝向
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import actionlib
import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_pixels import px2x, px2y, INNER        # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_FILE = os.path.join(WS, 'src', 'competition_robot', 'config', 'waypoints.yaml')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Patrol(object):
    def __init__(self, a):
        self.a = a
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.gt = None
        self.scan = None            # 最新一帧 /scan, 给激光对墙定位用
        self.trace = []
        rospy.Subscriber('/odom_groundtruth', Odometry, self.cb_gt, queue_size=50)
        rospy.Subscriber('/scan', LaserScan, self.cb_scan, queue_size=5)
        # ---- 识别联动: 到识别点位时请求一次识别, 并把结果打进本日志 ----
        #   ★ 识别是独立节点 (src/competition_robot/scripts/vision_detect.py) ——
        #     比赛方要的是 roslaunch 起世界 + rosrun 起节点, 不搞"一个脚本全包"。
        self.vision_pub = rospy.Publisher('/vision/request', String, queue_size=5)
        self.vision_res = None
        rospy.Subscriber('/vision/result', String, self.cb_vision, queue_size=5)

    def cb_vision(self, m):
        try:
            self.vision_res = json.loads(m.data)
        except Exception:                       # noqa: BLE001
            self.vision_res = None

    def ask_vision(self, name, task, index):
        """到识别点位后请求识别节点看一眼, 等结果并打印（拿不到就跳过, 不阻塞跑图）。"""
        if self.a.no_vision or not task or task == 'waypoint':
            return None
        kind = {'traffic_light': 'light', 'standee': 'standee',
                'plate': 'plate'}.get(task, 'all')
        self.vision_res = None
        self.vision_pub.publish(String(data=json.dumps(
            dict(point=name, kind=kind, index=index), ensure_ascii=False)))
        t0 = time.time()
        while self.vision_res is None and time.time() - t0 < self.a.vision_timeout:
            rospy.sleep(0.1)
        if self.vision_res is None:
            rospy.logwarn('     └ 没等到识别结果（vision_detect.py 起了吗? '
                          '超时 %.1fs）' % self.a.vision_timeout)
            return None
        for ln in self.vision_res.get('lines', []):
            rospy.loginfo('     └ %s' % ln)
        return self.vision_res

    LIGHT_CN = {'red': '红灯', 'yellow': '黄灯', 'green': '绿灯', 'none': '没看到灯'}

    def light_gate(self, name, index, first=None):
        """红绿灯**通行闸** —— 只有确认绿灯才返回 True。

        按交通规则（不是可选项）：
          * 红灯 / 黄灯 -> 原地停车等待，每隔 light_recheck 秒再看一次
          * 没看到灯   -> 当作"不能通行"同样等待（免得"看不见就默认走"）
          * 绿灯       -> 连续 light_confirm 次都读到绿灯才放行（防单帧误检）
        等超过 light_max_wait 仍未确认绿灯 -> 返回 False（本次运行到此为止），
        除非显式给了 --light-none-go 才放行。

        ★ 为什么要"连续确认": 识别节点一次请求抓 3 帧取最自信的一颗灯珠，
          已经有一点冗余；再加一层跨请求确认，能挡掉偶发误检。
        ★ 为什么这个闸必须在**离开路口前最后**判: 先判灯再去干别的活，等干完
          灯早变了（灯循环 绿6s->黄2s->红6s）。所以路线里红绿灯排在同地点其它站之后。
        """
        if self.a.no_light_gate:
            return True
        t0, res, confirm = time.time(), first, 0
        while not rospy.is_shutdown():
            st = ((res or {}).get('light') or {}).get('state', 'none')
            if st == 'green':
                confirm += 1
                if confirm >= max(1, self.a.light_confirm):
                    rospy.loginfo('  [交通灯] 绿灯（连续 %d 次确认）-> 放行' % confirm)
                    return True
                rospy.loginfo('  [交通灯] 绿灯（%d/%d 次确认）'
                              % (confirm, self.a.light_confirm))
            else:
                confirm = 0
                self.stop()
                rospy.loginfo('  [交通灯] %s -> 原地停车等待（已等 %.0f s）'
                              % (self.LIGHT_CN.get(st, st), time.time() - t0))
            if time.time() - t0 > self.a.light_max_wait:
                if self.a.light_none_go:
                    rospy.logwarn('  [交通灯] 等了 %.0f s 仍没确认绿灯；'
                                  '--light-none-go 已给 -> 放行' % (time.time() - t0))
                    return True
                rospy.logwarn('  [交通灯] 等了 %.0f s 仍没确认绿灯 -> 停车，'
                              '本次运行结束' % (time.time() - t0))
                return False
            rospy.sleep(self.a.light_recheck)
            res = self.ask_vision(name, 'traffic_light', index)
        return False

    def cb_scan(self, m):
        self.scan = m

    def cb_gt(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.gt = (p.x, p.y, yaw)
        self.trace.append((m.header.stamp.to_sec(), p.x, p.y, yaw))

    def pose(self):
        """机器人当前位姿 (map 系); 优先 TF, 拿不到就退回真值/里程计"""
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rospy.Time(0),
                                         rospy.Duration(1.0))
            p = t.transform.translation
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (p.x, p.y, yaw)
        except Exception as e:
            rospy.logwarn_throttle(5.0, '查 TF 失败: %s' % e)
            return self.gt

    def stop(self):
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.2)

    # ---------------------------------------------------------------- 激光对墙定位
    @staticmethod
    def _fit_axis_line(pts, axis):
        """最小二乘拟合一条"近轴"直线, 用来量墙。

        axis='x': 拟合 x = a*y + b  (竖墙; 位姿朝向正确时 a=0, b=墙的 x 坐标)
        axis='y': 拟合 y = a*x + b  (横墙; 位姿朝向正确时 a=0, b=墙的 y 坐标)
        返回 (a, b); 点太少或退化时返回 None。
        """
        n = len(pts)
        if n < 8:
            return None
        if axis == 'x':
            U = [p[0] for p in pts]
            V = [p[1] for p in pts]
        else:
            U = [p[1] for p in pts]
            V = [p[0] for p in pts]
        mv = sum(V) / n
        mu = sum(U) / n
        svv = sum((v - mv) ** 2 for v in V)
        if svv < 1e-9:
            return None
        a = sum((V[i] - mv) * (U[i] - mu) for i in range(n)) / svv
        return (a, mu - a * mv)

    def laser_wall_pose(self, inner=2.176, band=0.40, max_iter=3, min_pts=10):
        """用 2D 雷达"对墙"求一个**绝对**位姿 (只在墙边/角落有效)。

        思路: 地图已知墙在 ±inner。把扫描点按当前位姿投到 map 系后, 贴近墙面的
        那批点理应正好落在墙上。对这批点拟合直线:
            * 斜率 -> 位姿的**朝向**误差
            * 截距 -> 位姿的**位置**误差
        迭代几次即收敛。

        为什么不用 AMCL: 本场地只有四面墙, 方房间里沿墙滑移不可观测, AMCL 位置
        误差约 5~6cm。而这里直接用激光测距, 在角落处能得到厘米级以下的绝对位置,
        且不依赖里程计漂移。真机上同一路 /scan 可直接复用。

        返回 (x, y, yaw) 或 None (可用点太少 / 没有雷达数据)。
        """
        scan = self.scan
        rough = self.pose()
        if scan is None or rough is None:
            return None
        try:
            tr = self.tf.lookup_transform('map', scan.header.frame_id, rospy.Time(0),
                                          rospy.Duration(0.3))
        except Exception:
            return None

        # 雷达在车体坐标系里的固定偏移 (一次算好, 之后按候选位姿重新组装)
        bx, by, bth = rough
        dxw = tr.transform.translation.x - bx
        dyw = tr.transform.translation.y - by
        q = tr.transform.rotation
        lth = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(bth), math.sin(bth)
        off = (c * dxw + s * dyw, -s * dxw + c * dyw, wrap(lth - bth))

        ang = [scan.angle_min + i * scan.angle_increment for i in range(len(scan.ranges))]
        rr = list(scan.ranges)
        rmin, rmax = scan.range_min, scan.range_max

        def compose(pose):
            c2, s2 = math.cos(pose[2]), math.sin(pose[2])
            return (pose[0] + c2 * off[0] - s2 * off[1],
                    pose[1] + s2 * off[0] + c2 * off[1],
                    pose[2] + off[2])

        def collect(pose):
            lx, ly, lth2 = compose(pose)
            sx, sy = [], []
            for i, r in enumerate(rr):
                if not (rmin < r < rmax):
                    continue
                wx = lx + r * math.cos(lth2 + ang[i])
                wy = ly + r * math.sin(lth2 + ang[i])
                if abs(wx - wall_x) < band and abs(wy) < inner - 0.30:
                    sx.append((wx, wy))
                if abs(wy - wall_y) < band and abs(wx) < inner - 0.30:
                    sy.append((wx, wy))
            return sx, sy

        wall_x = inner if bx > 0 else -inner
        wall_y = inner if by > 0 else -inner

        pose = [bx, by, bth]
        for _ in range(max_iter):
            sx, sy = collect(pose)
            fx = self._fit_axis_line(sx, 'x')
            fy = self._fit_axis_line(sy, 'y')
            if fx and fy:
                # 竖墙 x=a*y+b 与横墙 y=c*x+d: 朝向误差 eps = (-a + c)/2
                eps = (-fx[0] + fy[0]) / 2.0
                pose[2] = wrap(pose[2] - eps)
            sx, sy = collect(pose)
            if len(sx) >= min_pts:
                pose[0] -= sum(p[0] for p in sx) / len(sx) - wall_x
            if len(sy) >= min_pts:
                pose[1] -= sum(p[1] for p in sy) / len(sy) - wall_y
            if len(sx) < min_pts and len(sy) < min_pts:
                return None
        return (pose[0], pose[1], pose[2])

    def pose_servo(self, tx, ty, tyaw, pos_tol=None, yaw_tol=None,
                   vmax=0.16, vymax=0.12, wmax=0.5, kp=1.2, kyaw=2.0,
                   reverse_only=True, timeout=45.0, label='park'):
        """位姿伺服: 不经过 move_base, 直接用 cmd_vel 闭环收敛到 (tx,ty,tyaw)。

        和 go_to() 的关键区别: **终止条件是"位姿到位", 不是"走了多少米"**。
        倒车入库就该这么做 —— 距离是结果而非输入, 换个库位只改目标位姿即可。

        位姿来源每帧先试"激光对墙"(绝对、精确), 拿不到就退回 AMCL/里程计。
        麦轮可以一边倒一边横移修正, 不用来回摆头。

        reverse_only=True 时纵向速度只允许 <= +0.02 (即只倒车, 不倒回去),
        符合倒车入库的语义。

        pos_tol 默认 0.012 m: 激光对墙的绝对精度约 3mm, 所以容差可以收到厘米级;
        实测收到这个值能稳定收敛 (再紧会因控制抖动反复)。
        """
        pos_tol = self.a.park_pos_tol if pos_tol is None else pos_tol
        yaw_tol = self.a.park_yaw_tol if yaw_tol is None else yaw_tol
        rate = rospy.Rate(20)
        t0 = rospy.Time.now()
        n_laser = n_amcl = 0
        err_xy = err_yaw = float('nan')
        while not rospy.is_shutdown():
            p = self.laser_wall_pose()
            if p is not None:
                n_laser += 1
            else:
                p = self.pose()
                n_amcl += 1
            if p is None:
                rate.sleep()
                continue
            ex, ey = tx - p[0], ty - p[1]
            eth = wrap(tyaw - p[2])
            err_xy, err_yaw = math.hypot(ex, ey), abs(eth)
            if err_xy < pos_tol and err_yaw < yaw_tol:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('  [%s] 位姿伺服超时: 剩 %.3f m / %.1f deg'
                              % (label, err_xy, math.degrees(err_yaw)))
                break
            # 误差投影到车体系: 目标在车后方时 exb < 0 -> vx < 0 -> 倒车
            c, s = math.cos(p[2]), math.sin(p[2])
            exb = c * ex + s * ey
            eyb = -s * ex + c * ey
            vx = max(-vmax, min(0.02 if reverse_only else vmax, kp * exb))
            vy = max(-vymax, min(vymax, kp * eyb))
            wz = max(-wmax, min(wmax, kyaw * eth))
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            self.cmd_pub.publish(t)
            rate.sleep()
        self.stop()
        rospy.loginfo('  [%s] 位姿伺服结束  终误差 %.4f m / %.2f deg  用时 %.1fs'
                      '  (激光 %d 帧 / AMCL %d 帧)'
                      % (label, err_xy, math.degrees(err_yaw),
                         (rospy.Time.now() - t0).to_sec(), n_laser, n_amcl))
        return err_xy < pos_tol and err_yaw < yaw_tol

    def rotate_to(self, target_yaw, tol=0.02, timeout=15.0):
        """原地转到 target_yaw (rad). 返回最终误差(rad)"""
        rate = rospy.Rate(20)
        t0 = rospy.Time.now()
        prev = None
        while not rospy.is_shutdown():
            p = self.pose()
            if p is None:
                rate.sleep(); continue
            err = wrap(target_yaw - p[2])
            if abs(err) < tol:
                break
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('  原地转向超时, 剩余误差 %.1f deg' % math.degrees(err))
                break
            w = max(-self.a.max_w, min(self.a.max_w, self.a.k_w * err))
            # 末端用更小的角速度, 才能停在 1 度以内 (20Hz 下 0.15rad/s 一步 0.43 度)
            lo = 0.12 if abs(err) < 0.15 else 0.25
            if abs(w) < lo:
                w = math.copysign(lo, w)
            t = Twist()
            t.angular.z = w
            self.cmd_pub.publish(t)
            prev = err
            rate.sleep()
        self.stop()
        p = self.pose()
        return abs(wrap(target_yaw - p[2])) if p else -1.0

    def send_goal(self, x, y, yaw, timeout):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = x
        g.target_pose.pose.position.y = y
        g.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.client.send_goal(g)
        ok = self.client.wait_for_result(rospy.Duration(timeout))
        st = self.client.get_state()
        return ok and st == actionlib.GoalStatus.SUCCEEDED, st

    def go_to(self, name, x, y, yaw_final=None):
        t0 = time.time()
        p = self.pose()
        if p is None:
            rospy.logerr('拿不到位姿, 跳过 %s' % name)
            return False
        bearing = math.atan2(y - p[1], x - p[0])
        if self.a.pre_rotate:
            self.rotate_to(bearing)
        # 默认**不**把最终朝向下给 move_base。否则规划器要在终点原地转一整圈,
        # 而终点一旦靠近墙, 转向余量小于定位误差就会被判成碰撞, 表现为偶发卡住/
        # 触发恢复行为。改成: move_base 只管开到位置(目标朝向=行进方向, 到点几乎
        # 不用转), 到点后的朝向完全交给 rotate_to() —— 纯 cmd_vel, 不经过规划器,
        # 180 度掉头也稳定。想要旧行为加 --goal-includes-yaw。
        goal_yaw = yaw_final if (yaw_final is not None and self.a.goal_includes_yaw) \
            else bearing
        ok, st = self.send_goal(x, y, goal_yaw, self.a.timeout)
        if ok:
            if yaw_final is not None:
                e_yaw = self.rotate_to(yaw_final)   # 到点后再把朝向摆正 (cmd_vel)
                rospy.loginfo('     └ 原地摆正到 %.1f deg, 残余 %.1f deg'
                              % (math.degrees(wrap(yaw_final)), math.degrees(e_yaw)))
            e_amcl = None
            g = self.pose()
            if g:
                e_amcl = math.hypot(g[0] - x, g[1] - y)
            e_gt = math.hypot(self.gt[0] - x, self.gt[1] - y) if self.gt else None
            rospy.loginfo('  ✓ %-8s 到点  用时 %4.1fs  AMCL误差 %.3f m%s'
                          % (name, time.time() - t0, e_amcl or -1,
                             ('  真值误差 %.3f m' % e_gt) if e_gt is not None else ''))
            return True
        rospy.logwarn('  ✗ %-8s 失败/超时 (action state=%d, %.1fs)' % (name, st, time.time() - t0))
        return False


def load(path):
    cfg = yaml.safe_load(open(path))
    wps = []
    for i, w in enumerate(cfg.get('waypoints', [])):
        if 'world' in w:
            x, y = float(w['world'][0]), float(w['world'][1])
        elif 'pixel' in w:
            x, y = px2x(w['pixel'][0]), px2y(w['pixel'][1])
        else:
            raise ValueError('第 %d 个航点既没有 world 也没有 pixel' % (i + 1))
        # ★ 保留 yaml 里的其它字段 (task / shot 等) —— capture_points.py 要用它
        #   判断"这个站要不要停车拍照"。以前只留 name/x/y/yaw, 下游拿不到。
        d = dict(w)
        d.update(name=w.get('name', 'P%d' % (i + 1)), x=x, y=y, yaw=w.get('yaw'))
        wps.append(d)
    start = cfg.get('start')
    start_yaw = cfg.get('start_yaw')
    # 回程目标可以和出生点不同: 出生点若在角落, 原地转向的几何余量太小,
    # move_base 会因为定位抖动把它判成碰撞而卡住。见 waypoints.yaml 的 return_to。
    ret = cfg.get('return_to') or start
    # 倒车入库: {from: 起倒航点名, to: [x, y, yaw]}
    park = cfg.get('reverse_park')
    park = dict(park) if park else None
    if park:
        # 把起倒点的**名字**解析成坐标 (库位前的那个航点)
        want = park.get('from')
        hit = [w for w in wps if w['name'] == want]
        if not hit:
            raise ValueError('reverse_park.from=%r 不在航点列表里' % want)
        park['from_xy'] = [hit[0]['x'], hit[0]['y']]
        if len(park.get('to', [])) != 3:
            raise ValueError('reverse_park.to 必须是 [x, y, yaw] 三项')
    return (wps,
            ([float(start[0]), float(start[1])] if start else None),
            (float(start_yaw) if start_yaw is not None else None),
            ([float(ret[0]), float(ret[1])] if ret else None),
            park)


# 底盘外接半径 + footprint_padding, 由 costmap_common_params.yaml 的 footprint 算得。
# 原地转向时车心到墙至少要留这么多, 否则会被判成碰撞。
FOOTPRINT_R = 0.2582
SAFE_MARGIN = 0.10          # 希望额外留下的余量 (AMCL 定位误差实测中位 5.1 cm)


def clearance_note(x, y):
    """这个点原地转向还剩多少几何余量 (只考虑场地外墙)。"""
    margin = (INNER - max(abs(x), abs(y))) - FOOTPRINT_R
    if margin < 0:
        return '  ✗ 转不开 (余量 %+.3f m)' % margin
    if margin < SAFE_MARGIN:
        return '  ⚠ 旋转余量仅 %.3f m (定位误差就可能吃掉)' % margin
    return '  ✓ 旋转余量 %.3f m' % margin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=DEFAULT_FILE)
    ap.add_argument('--dry-run', action='store_true', help='只打印路线')
    ap.add_argument('--no-return-start', action='store_true')
    ap.add_argument('--no-pre-rotate', dest='pre_rotate', action='store_false')
    ap.add_argument('--loop', action='store_true')
    ap.add_argument('--timeout', type=float, default=60.0, help='每段超时(仿真秒)')
    ap.add_argument('--no-vision', action='store_true',
                    help='不做识别联动（不发 /vision/request）')
    ap.add_argument('--no-light-gate', action='store_true',
                    help='关掉红绿灯通行闸（默认开: 红/黄灯停车等待, 连续确认绿灯才走）')
    ap.add_argument('--light-recheck', type=float, default=1.5,
                    help='等灯时每隔几秒重新识别一次')
    ap.add_argument('--light-confirm', type=int, default=2,
                    help='连续几次读到绿灯才放行（防单帧误检）')
    ap.add_argument('--light-max-wait', type=float, default=60.0,
                    help='等灯上限(秒); 超时默认停车并结束本次运行')
    ap.add_argument('--light-none-go', action='store_true',
                    help='等超时后放行（默认不放行 —— 交通规则优先）')
    ap.add_argument('--vision-timeout', type=float, default=6.0,
                    help='等识别结果的超时 (s)')
    ap.add_argument('--max-w', type=float, default=1.0, help='原地转向最大角速度')
    ap.add_argument('--k-w', type=float, default=1.2, help='转向 P 增益')
    ap.add_argument('--save-trace', help='把轨迹存成 csv')
    ap.add_argument('--goal-includes-yaw', dest='goal_includes_yaw', action='store_true',
                    help='把最终朝向一起下给 move_base (旧行为)。默认不下: 先开到点, '
                         '朝向交给 cmd_vel 原地摆正 —— 不经过规划器, 贴墙也不会卡')
    ap.add_argument('--no-park', dest='do_park', action='store_false',
                    help='忽略配置里的 reverse_park, 不做倒车入库')
    ap.add_argument('--park-pos-tol', dest='park_pos_tol', type=float, default=0.012,
                    help='倒车入库的位置容差(m), 默认 0.012。激光对墙精度约 3mm, '
                         '所以可以收到厘米级; 再紧会因控制抖动反复')
    ap.add_argument('--park-yaw-tol', dest='park_yaw_tol', type=float, default=0.02,
                    help='倒车入库的朝向容差(rad), 默认 0.02 (~1.1 度)')
    ap.set_defaults(pre_rotate=True, goal_includes_yaw=False, do_park=True)
    a = ap.parse_args()

    wps, start, start_yaw, return_to, park = load(a.file)
    print('航点 %d 个 (来自 %s)' % (len(wps), a.file))
    for w in wps:
        print('   %-8s (%+.3f, %+.3f)%s'
              % (w['name'], w['x'], w['y'], clearance_note(w['x'], w['y'])))
    if park:
        px, py = float(park['to'][0]), float(park['to'][1])
        # 配置里也能覆盖容差 (不写就用命令行/默认值)
        a.park_pos_tol = float(park.get('pos_tol', a.park_pos_tol))
        a.park_yaw_tol = float(park.get('yaw_tol', a.park_yaw_tol))
        print('   %-8s (%+.3f, %+.3f)%s   ← 倒车入库: 从 %s 倒到这里, 朝向 %.1f deg'
              '  (容差 %.3fm/%.1f°)'
              % ('park', px, py, clearance_note(px, py),
                 park.get('from', '?'), math.degrees(float(park['to'][2])),
                 a.park_pos_tol, math.degrees(a.park_yaw_tol)))
    elif return_to:
        print('   %-8s (%+.3f, %+.3f)%s   ← 最后回到这里摆正朝向'
              % ('return', return_to[0], return_to[1], clearance_note(*return_to)))
    if a.dry_run:
        return

    rospy.init_node('patrol', anonymous=True)
    p = Patrol(a)
    rospy.loginfo('等 move_base ...')
    if not p.client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr('没有 move_base: 先 roslaunch competition_robot navigation.launch')
        sys.exit(1)

    # 出发前的朝向 (巡航结束后要转回来)
    p0 = None
    for _ in range(50):
        p0 = p.pose()
        if p0:
            break
        rospy.sleep(0.2)
    yaw_home = start_yaw if start_yaw is not None else (p0[2] if p0 else 0.0)
    rospy.loginfo('出发朝向 %.1f deg (巡航结束会转回这个朝向)'
                  % math.degrees(wrap(yaw_home)))

    ok_n = 0
    aborted = False
    try:
        while not rospy.is_shutdown():
            for w in wps:
                if p.go_to(w['name'], w['x'], w['y'], w['yaw']):
                    # 到识别点位 -> 让视觉节点看一眼（点位名去掉序号前缀）
                    pname = str(w['name']).split('_', 1)[-1]
                    pidx = str(w['name']).split('_', 1)[0]
                    res = p.ask_vision(pname, w.get('task'), pidx)
                    # ★ 红绿灯是通行闸: 只有确认绿灯才准继续开（交通规则必须遵守）
                    if w.get('task') == 'traffic_light' \
                            and not p.light_gate(pname, pidx, res):
                        aborted = True
                        break
                    ok_n += 1
            if aborted:
                rospy.logwarn('  [交通灯] 没有确认绿灯 -> 停在原地，本次运行结束'
                              '（不倒车入库）')
                break
            if park and a.do_park:
                # ---- 倒车入库 ----
                # 1) 先用 move_base(DWA) 开到"起倒点"(库里配的 from, 一般是最后一个航点)
                fx, fy = float(park['from_xy'][0]), float(park['from_xy'][1])
                tx, ty = float(park['to'][0]), float(park['to'][1])
                tyaw = float(park['to'][2])
                p.go_to(park.get('from', 'from'), fx, fy)
                # 2) 原地转到"背离库位"的朝向, 这样接下来是纯倒车
                back_yaw = math.atan2(fy - ty, fx - tx)
                p.rotate_to(back_yaw)
                # 3) 位姿伺服倒进去 (位姿优先用激光对墙, 不靠定时/定距)
                converged = p.pose_servo(tx, ty, tyaw, label='park')
                # 4) 和真值比一下, 看这次入位到底有多准
                if p.gt:
                    gx, gy = p.gt[0] - tx, p.gt[1] - ty
                    rospy.loginfo('  [park] 对真值: 位置 %.4f m, 朝向 %.2f deg  %s'
                                  % (math.hypot(gx, gy),
                                     math.degrees(abs(wrap(p.gt[2] - tyaw))),
                                     '✓ 入位' if converged else '✗ 未收敛'))
                rospy.loginfo('一圈跑完: %d/%d 个航点成功 + 倒车入库'
                              % (ok_n, len(wps)))
            else:
                if return_to and not a.no_return_start:
                    p.go_to('return', return_to[0], return_to[1], yaw_home)
                try:
                    p.vision_pub.publish(String(data=''))       # 让识别节点打印/落盘汇总
                    rospy.sleep(0.5)
                except Exception:                               # noqa: BLE001
                    pass
                rospy.loginfo('一圈跑完: %d/%d 个航点成功' % (ok_n, len(wps)))
            if not a.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        p.stop()
        if a.save_trace and p.trace:
            with open(a.save_trace, 'w') as f:
                f.write('t,x,y,yaw\n')
                for r in p.trace:
                    f.write('%.3f,%.4f,%.4f,%.4f\n' % r)
            print('轨迹已存 ->', a.save_trace)


if __name__ == '__main__':
    main()
