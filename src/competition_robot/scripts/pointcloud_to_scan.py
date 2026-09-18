#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pointcloud_to_scan.py —— 把 3D 点云切一层出来, 生成 2D 的 /scan。

3D 雷达只发点云, 但 AMCL / move_base 要 sensor_msgs/LaserScan;
系统里没装 pointcloud_to_laserscan, 所以这里自己切一层。

做法: 取雷达上下 [height_min, height_max] 高度带内的点,
      按方位角分 bin, 每个 bin 取最近的那个距离。

★ 同时支持两种点云消息:
    sensor_msgs/PointCloud2  (新, 一般的 3D 雷达驱动用这个)
    sensor_msgs/PointCloud   (老, gazebo_ros_block_laser 插件发的是这个)
  用 rospy.AnyMsg 订阅, 按实际类型反序列化, 所以换数据源不用改代码。

★ 车体自身回波过滤 (self filter) —— 这个不做的话 SLAM 一定歪:
    3D 雷达装得比车顶高, 它的**下俯光束会打到自己车顶**。实测这块回波落在
    车体局部 147°~213° 的一个扇形里, 距离恒定 ~0.207 m (车顶面)。
    近处更小的那些回波被 range_min 滤掉了, 正好剩下这一圈 —— 于是 /scan 里
    多出一面**跟着车走的假墙**。SLAM 会拼命把这面假墙和上一帧画的假墙对上,
    结果就是: 轨迹走一半、yaw 越跑越偏、建出来的图整体歪掉 (实测直行 1.7 m,
    SLAM 只前进 0.82 m, 还偏了 16°)。
    办法: 把落在**车体碰撞盒**内部的点直接丢掉。盒子由 robot_params.yaml
    里的 chassis 尺寸 + 雷达安装位置算出来, 所以挪雷达/改车体后自动跟着变。

参数:
    ~input / ~output        话题名
    ~height_min/max         高度带 (相对雷达坐标系, m)
    ~angle_min_deg/max_deg  角度范围, 默认整圈
    ~angle_step_deg         角分辨率
    ~range                  有效距离 [min, max]
    ~self_filter            车体盒子 [xmin,xmax,ymin,ymax,zmin,zmax] (雷达系), 手动覆盖
    ~self_filter_margin     盒子外扩量, 默认 0.01 m
    ~self_filter_from_robot 默认 true: 从 /robot_params 自动算车体盒子
    ~output_cloud           可选: 发布过滤后的点云 (PointCloud2) 给 RViz 看
"""

from __future__ import annotations

import math

import numpy as np
import rospy
from rospy.msg import AnyMsg
from sensor_msgs.msg import LaserScan, PointCloud, PointCloud2
from sensor_msgs import point_cloud2 as pc2


class PointCloudToScan(object):
    def __init__(self):
        self.input = rospy.get_param('~input', 'points')
        self.output = rospy.get_param('~output', 'scan')
        self.zmin = float(rospy.get_param('~height_min', -0.15))
        self.zmax = float(rospy.get_param('~height_max', 0.15))
        self.amin = math.radians(float(rospy.get_param('~angle_min_deg', -180.0)))
        self.amax = math.radians(float(rospy.get_param('~angle_max_deg', 180.0)))
        step = math.radians(float(rospy.get_param('~angle_step_deg', 0.5)))
        rng = rospy.get_param('~range', [0.2, 30.0])
        self.rmin, self.rmax = float(rng[0]), float(rng[1])

        self.n = max(1, int(round((self.amax - self.amin) / step)))
        self.inc = (self.amax - self.amin) / self.n
        self.rate = float(rospy.get_param('~rate', 10.0))

        # ---- 车体自身回波过滤 ----
        self.sf_lo, self.sf_hi = self._self_filter_box()
        self.sf_dropped = 0
        self.out_cloud = rospy.get_param('~output_cloud', '')
        self.cloud_pub = (rospy.Publisher(self.out_cloud, PointCloud2, queue_size=1)
                          if self.out_cloud else None)

        self.pub = rospy.Publisher(self.output, LaserScan, queue_size=10)
        self.sub = rospy.Subscriber(self.input, AnyMsg, self.cb, queue_size=1)
        self.seen_types = set()
        rospy.loginfo('pointcloud_to_scan: %s -> %s, %d 个 bin, 高度带 [%.2f, %.2f], 距离 [%.2f, %.2f]'
                      % (self.input, self.output, self.n, self.zmin, self.zmax,
                         self.rmin, self.rmax))
        if self.sf_lo is not None:
            rospy.loginfo('pointcloud_to_scan: 车体自身回波过滤 [%.3f,%.3f]x[%.3f,%.3f]x[%.3f,%.3f] (雷达系)'
                          % (self.sf_lo[0], self.sf_hi[0], self.sf_lo[1], self.sf_hi[1],
                             self.sf_lo[2], self.sf_hi[2]))
        else:
            rospy.logwarn('pointcloud_to_scan: 没有车体自身回波过滤, 雷达打到车顶会变成假墙')

    # ------------------------------------------------------------ 车体盒子
    @staticmethod
    def _rpy_to_R(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr]]

    def _self_filter_box(self):
        """返回 (lo, hi): 车体碰撞盒在**雷达坐标系**下的包围盒。"""
        manual = rospy.get_param('~self_filter', None)
        margin = float(rospy.get_param('~self_filter_margin', 0.01))
        if manual:
            lo = [float(manual[i]) - margin for i in (0, 2, 4)]
            hi = [float(manual[i]) + margin for i in (1, 3, 5)]
            return lo, hi
        if not bool(rospy.get_param('~self_filter_from_robot', True)):
            return None, None
        try:
            p = rospy.get_param('/robot_params')
            ch = p['chassis']
            lid = p['sensors']['lidar3d']
            mount = [float(v) for v in lid['mount']]
            rpy = [float(v) for v in lid.get('mount_rpy', [0.0, 0.0, 0.0])]
            center = [float(v) for v in ch.get('center_offset', [0.0, 0.0, 0.0])]
            size = [float(ch['length']), float(ch['width']), float(ch['height'])]
        except Exception as exc:                      # 参数不全就不做过滤
            rospy.logwarn('pointcloud_to_scan: 算不出车体盒子 (%s)' % exc)
            return None, None
        R = self._rpy_to_R(*rpy)                      # 雷达在 base_link 下的姿态
        half = [s / 2.0 for s in size]
        vals = [[1e9] * 3, [-1e9] * 3]
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    b = [center[0] + sx * half[0], center[1] + sy * half[1],
                         center[2] + sz * half[2]]
                    d = [b[i] - mount[i] for i in range(3)]
                    # p_laser = R^T (p_base - mount)
                    q = [sum(R[k][i] * d[k] for k in range(3)) for i in range(3)]
                    for i in range(3):
                        vals[0][i] = min(vals[0][i], q[i])
                        vals[1][i] = max(vals[1][i], q[i])
        lo = [vals[0][i] - margin for i in range(3)]
        hi = [vals[1][i] + margin for i in range(3)]
        return lo, hi

    def _drop_self(self, pts):
        if self.sf_lo is None or pts.size == 0:
            return pts
        lo, hi = self.sf_lo, self.sf_hi
        inside = ((pts[:, 0] >= lo[0]) & (pts[:, 0] <= hi[0]) &
                  (pts[:, 1] >= lo[1]) & (pts[:, 1] <= hi[1]) &
                  (pts[:, 2] >= lo[2]) & (pts[:, 2] <= hi[2]))
        k = int(inside.sum())
        if k:
            self.sf_dropped += k
            pts = pts[~inside]
        return pts

    # ------------------------------------------------------------ 点云 -> Nx3 数组
    @staticmethod
    def _points_from_pc2(msg):
        arr = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        return np.array(arr, dtype=np.float64).reshape(-1, 3) if arr else None

    @staticmethod
    def _points_from_pc1(msg):
        pts = msg.points
        if not pts:
            return None
        return np.array([[p.x, p.y, p.z] for p in pts], dtype=np.float64).reshape(-1, 3)

    def cb(self, raw):
        # rospy.AnyMsg 的 _type 永远是 '*', 真实类型在连接头里
        try:
            t = raw._connection_header.get('type', '')
        except AttributeError:
            t = getattr(raw, '_type', '')
        try:
            if t == 'sensor_msgs/PointCloud2':
                msg = PointCloud2()
                msg.deserialize(raw._buff)
                pts = self._points_from_pc2(msg)
            elif t == 'sensor_msgs/PointCloud':
                msg = PointCloud()
                msg.deserialize(raw._buff)
                pts = self._points_from_pc1(msg)
            else:
                if t not in self.seen_types:
                    self.seen_types.add(t)
                    rospy.logwarn('pointcloud_to_scan: 不支持的点云类型 %s' % t)
                return
        except Exception as exc:
            rospy.logwarn_throttle(10.0, 'pointcloud_to_scan: 解析失败: %s' % exc)
            return

        if pts is None or pts.size == 0:
            return

        # ★ 先丢掉打在自己车体上的点 —— 不做的话会多出一面跟着车走的假墙
        pts = self._drop_self(pts)
        if pts.size == 0:
            return

        # 高度带过滤
        pts = pts[(pts[:, 2] >= self.zmin) & (pts[:, 2] <= self.zmax)]
        if pts.size == 0:
            return

        if self.cloud_pub is not None:
            cloud = PointCloud2()
            cloud.header = msg.header
            self.cloud_pub.publish(pc2.create_cloud_xyz32(cloud.header,
                                                          pts.astype(np.float32)))

        r = np.hypot(pts[:, 0], pts[:, 1])
        ang = np.arctan2(pts[:, 1], pts[:, 0])

        m = (r >= self.rmin) & (r <= self.rmax)
        r, ang = r[m], ang[m]
        if r.size == 0:
            return

        idx = np.floor((ang - self.amin) / self.inc).astype(np.int64)
        valid = (idx >= 0) & (idx < self.n)
        idx, r = idx[valid], r[valid]
        if r.size == 0:
            return

        # 无回波的 bin 保持 inf —— ROS 的惯例 (ranges[i] = inf 表示没有回波)
        ranges = np.full(self.n, np.inf)
        np.minimum.at(ranges, idx, r)

        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = float(self.amin)
        scan.angle_max = float(self.amax)
        scan.angle_increment = float(self.inc)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(self.rate, 1e-3)
        scan.range_min = float(self.rmin)
        scan.range_max = float(self.rmax)
        scan.ranges = ranges.astype(np.float32).tolist()
        scan.intensities = []
        self.pub.publish(scan)


def main():
    rospy.init_node('pointcloud_to_scan')
    PointCloudToScan()
    rospy.spin()


if __name__ == '__main__':
    main()
