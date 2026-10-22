#!/usr/bin/env python3
"""Score /pat/estimate/los_error against /pat/sim/truth.

Matches messages by header stamp (the plant stamps IMU and truth identically
each tick, and the estimator stamps its output with the IMU stamp). Prints
error statistics after `--duration` seconds, skipping the first `--settle`.

    ros2 run pat_sim score_estimate.py --duration 10 --settle 2
"""

import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from pat_msgs.msg import LosEstimate, SimTruth

MODE_NAMES = {0: 'IMU_AND_CAMERA', 1: 'CAMERA_ONLY', 2: 'IMU_ONLY'}


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


class Scorer(Node):
    def __init__(self, settle_s):
        super().__init__('score_estimate')
        self.settle_ns = int(settle_s * 1e9)
        self.t0 = None
        self.truth = {}
        self.pending = []
        self.err = []
        self.err_valid = []
        self.sigma_ratio = []
        self.valid_flags = []
        self.cam_age = []
        self.modes = {}
        self.last = None
        self.count = 0
        # Same policies as pat_common/qos.hpp (deadline on the estimate topic).
        est_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             deadline=Duration(nanoseconds=2_000_000))
        truth_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LosEstimate, '/pat/estimate/los_error', self.on_est, est_qos)
        self.create_subscription(SimTruth, '/pat/sim/truth', self.on_truth, truth_qos)

    def on_truth(self, m):
        self.truth[stamp_ns(m.header)] = (m.ex, m.ey)
        if len(self.truth) > 20000:
            for k in sorted(self.truth)[:5000]:
                del self.truth[k]

    def on_est(self, m):
        s = stamp_ns(m.header)
        if self.t0 is None:
            self.t0 = s
        self.count += 1
        self.last = m
        self.pending.append((s, m))
        # Truth may arrive slightly after the estimate; match with a lag.
        keep = []
        for ps, pm in self.pending:
            if ps in self.truth:
                self.record(ps, pm, self.truth[ps])
            elif s - ps < 200_000_000:
                keep.append((ps, pm))
        self.pending = keep

    def record(self, s, m, tr):
        if s - self.t0 < self.settle_ns:
            return
        e = math.hypot(m.ex - tr[0], m.ey - tr[1])
        self.err.append(e)
        self.valid_flags.append(m.valid)
        if m.valid:
            self.err_valid.append(e)
            sig = math.sqrt(max(m.covariance[0] + m.covariance[5], 1e-12))
            self.sigma_ratio.append(e / sig)
        self.cam_age.append(m.camera_age_ms)
        self.modes[m.mode] = self.modes.get(m.mode, 0) + 1

    def report(self, mode_label):
        if not self.err:
            print('no matched samples', file=sys.stderr)
            return 1
        rms = math.sqrt(sum(e * e for e in self.err) / len(self.err))
        rms_v = math.sqrt(sum(e * e for e in self.err_valid) / len(self.err_valid)) if self.err_valid else float('nan')
        p99 = sorted(self.err)[int(0.99 * (len(self.err) - 1))]
        valid_frac = sum(self.valid_flags) / len(self.valid_flags)
        ages = sorted(self.cam_age)
        m = self.last
        print(f'--- score ({mode_label}) ---')
        print(f'matched samples      : {len(self.err)}  (estimates received {self.count})')
        print(f'error  RMS / p99 / max: {rms:.2f} / {p99:.2f} / {max(self.err):.2f} urad')
        print(f'error  RMS while valid: {rms_v:.2f} urad  (valid {100*valid_frac:.1f} % of samples)')
        if self.sigma_ratio:
            print(f'|err|/sigma median    : {statistics.median(self.sigma_ratio):.2f}  '
                  f'(~0.8-1.2 means the covariance is honest)')
        print(f'camera_age_ms  p50/p99: {ages[len(ages)//2]:.1f} / {ages[int(0.99*(len(ages)-1))]:.1f}')
        print('modes                 : ' + ', '.join(
            f'{MODE_NAMES.get(k, k)} {100*v/len(self.err):.1f} %' for k, v in sorted(self.modes.items())))
        print(f'camera frames accepted/rejected: {m.camera_frames_accepted} / {m.camera_frames_rejected}; '
              f'imu samples rejected: {m.imu_samples_rejected}')
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=10.0)
    ap.add_argument('--settle', type=float, default=2.0)
    ap.add_argument('--label', default='')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    n = Scorer(args.settle)
    end = time.monotonic() + args.duration + args.settle
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    rc = n.report(args.label or 'unlabelled')
    n.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
