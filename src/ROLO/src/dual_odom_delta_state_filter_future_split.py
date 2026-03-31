#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_inverse,
    quaternion_matrix,
    quaternion_multiply,
)
from visualization_msgs.msg import Marker


# ============================================================
# helpers
# ============================================================

def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


def wrap_vec_angles(vec, angle_idx):
    out = np.array(vec, dtype=float)
    for idx in angle_idx:
        out[idx] = wrap_angle(out[idx])
    return out


def quat_to_ypr(quat_xyzw):
    """
    返回 yaw, pitch, roll
    tf 的 euler_from_quaternion 返回 roll, pitch, yaw
    """
    roll, pitch, yaw = euler_from_quaternion(quat_xyzw)
    return yaw, pitch, roll


def ypr_to_quat(yaw, pitch, roll):
    """
    tf 的 quaternion_from_euler 输入顺序是 roll, pitch, yaw
    """
    q = quaternion_from_euler(roll, pitch, yaw)
    return np.array([q[0], q[1], q[2], q[3]], dtype=float)


def normalize_quat(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def quat_to_rotmat(q):
    return quaternion_matrix(q)[0:3, 0:3]


def apply_deadband_delta(delta, linear_deadband=0.0, angular_deadband=0.0):
    out = np.array(delta, dtype=float)
    if linear_deadband > 0.0:
        for i in range(3):
            if abs(out[i]) < linear_deadband:
                out[i] = 0.0
    if angular_deadband > 0.0:
        for i in range(3, 6):
            if abs(out[i]) < angular_deadband:
                out[i] = 0.0
    return out


def delta_to_rate(delta, dt):
    if dt <= 1e-9:
        return np.zeros(6, dtype=float)
    out = np.asarray(delta, dtype=float) / dt
    return out


def compute_relative_delta(prev_position, prev_orientation, curr_position, curr_orientation):
    """
    计算相邻两帧之间的相对变换（局部坐标系）：

        Delta = [dx_local, dy_local, dz_local, dyaw, dpitch, droll]

    这里的平移是在 prev pose 的局部坐标系里表达，
    旋转是 q_rel = q_prev^{-1} * q_curr 再转成 yaw/pitch/roll 增量。
    """
    prev_position = np.asarray(prev_position, dtype=float)
    curr_position = np.asarray(curr_position, dtype=float)
    prev_orientation = normalize_quat(prev_orientation)
    curr_orientation = normalize_quat(curr_orientation)

    dp_world = curr_position - prev_position
    R_prev = quat_to_rotmat(prev_orientation)
    dp_local = R_prev.T.dot(dp_world)

    q_rel = quaternion_multiply(quaternion_inverse(prev_orientation), curr_orientation)
    q_rel = normalize_quat(q_rel)
    dyaw, dpitch, droll = quat_to_ypr(q_rel)

    delta = np.array([
        dp_local[0],
        dp_local[1],
        dp_local[2],
        wrap_angle(dyaw),
        wrap_angle(dpitch),
        wrap_angle(droll),
    ], dtype=float)
    return delta


def apply_local_delta_to_pose(position, orientation, delta):
    """
    把局部增量 Delta 施加到当前 pose 上：

        T_new = T_old * T_delta

    其中 delta 的平移在 old pose 的局部坐标系里表达。
    """
    position = np.asarray(position, dtype=float)
    orientation = normalize_quat(orientation)
    delta = np.asarray(delta, dtype=float)

    dp_local = delta[0:3]
    dyaw, dpitch, droll = delta[3], delta[4], delta[5]

    R = quat_to_rotmat(orientation)
    new_position = position + R.dot(dp_local)

    q_delta = ypr_to_quat(dyaw, dpitch, droll)
    new_orientation = quaternion_multiply(orientation, q_delta)
    new_orientation = normalize_quat(new_orientation)

    return new_position, new_orientation


# ============================================================
# measurement container
# ============================================================

class OdomMeasurement(object):
    def __init__(self, timestamp, position, orientation, linear_velocity, angular_velocity, source):
        self.timestamp = float(timestamp)
        self.position = np.asarray(position, dtype=float)
        self.orientation = normalize_quat(orientation)
        self.linear_velocity = np.asarray(linear_velocity, dtype=float)
        self.angular_velocity = np.asarray(angular_velocity, dtype=float)
        self.source = source


def odom_msg_to_measurement(msg, source):
    return OdomMeasurement(
        timestamp=msg.header.stamp.to_sec(),
        position=[
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ],
        orientation=[
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ],
        linear_velocity=[
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ],
        angular_velocity=[
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ],
        source=source,
    )


# ============================================================
# increment-state Kalman filter
# ============================================================

class DeltaStateKalmanFilter(object):
    """
    状态定义（12 维）：
        x = [dx, dy, dz, dyaw, dpitch, droll,
             ddx, ddy, ddz, ddyaw, ddpitch, ddroll]

    含义：
    - 前 6 维：当前这一小段“相邻两帧之间的相对变换 Delta”
    - 后 6 维：Delta 的导数 Delta_dot

    控制输入（6 维）：
        u = Delta_dot

    观测（6 维）：
        z = Delta

    这正对应用户要求：
    - 状态 = “两帧之间的变换 + 变换导数”
    - 控制 = “变化量的导数”
    - 位姿输出 = 由滤波后的 Delta 逐步迭代累积得到
    """
    def __init__(self, Q, R_front, R_back, control_alpha=0.85, cov_min_eig=1e-9):
        self.n = 12
        self.m = 6

        self.Q = np.asarray(Q, dtype=float)
        self.R_front = np.asarray(R_front, dtype=float)
        self.R_back = np.asarray(R_back, dtype=float)
        self.control_alpha = float(control_alpha)
        self.cov_min_eig = float(cov_min_eig)

        self.angle_idx_state_delta = [3, 4, 5]
        self.angle_idx_measurement = [3, 4, 5]

        self.I12 = np.eye(12, dtype=float)
        self.I6 = np.eye(6, dtype=float)

    def project_cov(self, P):
        P = 0.5 * (np.asarray(P, dtype=float) + np.asarray(P, dtype=float).T)
        try:
            w, v = np.linalg.eigh(P)
        except np.linalg.LinAlgError:
            P = P + self.cov_min_eig * np.eye(P.shape[0])
            w, v = np.linalg.eigh(P)
        w = np.maximum(w, self.cov_min_eig)
        P = v.dot(np.diag(w)).dot(v.T)
        return 0.5 * (P + P.T)

    def _build_Qd(self, dt):
        dt_eff = max(float(dt), 1e-3)
        Qd = np.array(self.Q, dtype=float)
        Qd[0:6, 0:6] *= dt_eff * dt_eff
        Qd[6:12, 6:12] *= dt_eff
        return self.project_cov(Qd)

    def predict(self, x, P, dt, u=None):
        dt = float(dt)
        x = np.asarray(x, dtype=float)
        P = self.project_cov(P)

        if u is None:
            u = x[6:12].copy()
        u = np.asarray(u, dtype=float)

        a = self.control_alpha

        F = np.zeros((12, 12), dtype=float)
        B = np.zeros((12, 6), dtype=float)

        # delta_k+1 = delta_k + ((1-a) * delta_dot_k + a * u_k) * dt
        F[0:6, 0:6] = self.I6
        F[0:6, 6:12] = (1.0 - a) * dt * self.I6
        B[0:6, :] = a * dt * self.I6

        # delta_dot_k+1 = (1-a) * delta_dot_k + a * u_k
        F[6:12, 6:12] = (1.0 - a) * self.I6
        B[6:12, :] = a * self.I6

        x_pred = F.dot(x) + B.dot(u)
        x_pred[0:6] = wrap_vec_angles(x_pred[0:6], self.angle_idx_state_delta)

        Qd = self._build_Qd(dt)
        P_pred = F.dot(P).dot(F.T) + Qd
        P_pred = self.project_cov(P_pred)
        return x_pred, P_pred

    def update_delta(self, x, P, z, R):
        x = np.asarray(x, dtype=float)
        P = self.project_cov(P)
        z = np.asarray(z, dtype=float)
        R = self.project_cov(R)

        H = np.zeros((6, 12), dtype=float)
        H[:, 0:6] = self.I6

        z_pred = x[0:6].copy()
        y = z - z_pred
        y = wrap_vec_angles(y, self.angle_idx_measurement)

        S = H.dot(P).dot(H.T) + R
        S = self.project_cov(S)

        PHt = P.dot(H.T)
        try:
            K = np.linalg.solve(S.T, PHt.T).T
        except np.linalg.LinAlgError:
            K = PHt.dot(np.linalg.pinv(S))

        x_new = x + K.dot(y)
        x_new[0:6] = wrap_vec_angles(x_new[0:6], self.angle_idx_state_delta)

        # Joseph form for better numerical stability
        I_KH = self.I12 - K.dot(H)
        P_new = I_KH.dot(P).dot(I_KH.T) + K.dot(R).dot(K.T)
        P_new = self.project_cov(P_new)
        return x_new, P_new


# ============================================================
# node
# ============================================================

class DualOdomDeltaStateNode(object):
    def __init__(self):
        # -------------------------
        # topics / frames / timers
        # -------------------------
        self.front_topic = rospy.get_param("~front_topic", "/odometry/lidar_incremental")
        self.back_topic = rospy.get_param("~back_topic", "/rolo/mapping/odometry_incremental")
        self.output_topic = rospy.get_param("~output_topic", "/fused_odom")

        self.world_frame = rospy.get_param("~world_frame", "odometry")
        self.child_frame = rospy.get_param("~child_frame", "base_link")

        self.fusion_rate = rospy.get_param("~fusion_rate", 100.0)
        self.publish_rate = rospy.get_param("~publish_rate", 20.0)
        self.publish_use_now_stamp = rospy.get_param("~publish_use_now_stamp", True)
        self.publish_predict_to_now = rospy.get_param("~publish_predict_to_now", True)

        # 未来预测发布参数
        # 1) publish_predict_extra_sec:
        #    在“当前发布基准时刻”之上，再额外向未来预测多少秒。
        # 2) publish_predict_distance:
        #    如果 > 0，则继续向未来预测，直到预测位置相对“当前发布基准位置”的直线距离
        #    大于等于该阈值（米）为止。
        # 3) 如果 extra_sec 和 distance 同时开启，则取满足二者的更大预测时长。
        self.publish_predict_extra_sec = rospy.get_param("~publish_predict_extra_sec", 1)
        self.publish_predict_distance = rospy.get_param("~publish_predict_distance", 5)
        self.publish_predict_step_sec = rospy.get_param("~publish_predict_step_sec", 0.02)
        self.publish_predict_max_sec = rospy.get_param("~publish_predict_max_sec", 10.0)

        # 未来预测输出参数
        # 设计目标：
        # 1) /fused_odom 仍然只表示“预测到现在”的全局 odom，不再承载未来目标
        # 2) 额外发布未来目标的全局 odom
        # 3) 再把这个未来全局目标转换成“以当前时刻为原点”的局部 odom 发布
        self.future_global_output_topic = rospy.get_param("~future_global_output_topic", "/fused_odom_future_global")
        self.publish_future_global_odom = rospy.get_param("~publish_future_global_odom", True)

        self.future_local_output_topic = rospy.get_param("~future_local_output_topic", "/fused_odom_future_local")
        self.future_local_frame = rospy.get_param("~future_local_frame", "base_link")
        self.future_local_child_frame = rospy.get_param("~future_local_child_frame", self.child_frame + "_future")
        self.publish_future_local_odom = rospy.get_param("~publish_future_local_odom", True)

        # 可视化 topic
        self.marker_topic = rospy.get_param("~marker_topic", "/fused_odom_marker")
        self.future_global_marker_topic = rospy.get_param("~future_global_marker_topic", "/fused_odom_future_global_marker")
        self.future_local_marker_topic = rospy.get_param("~future_local_marker_topic", "/fused_odom_future_local_marker")
        self.marker_scale = rospy.get_param("~marker_scale", 1.0)
        self.marker_line_width = rospy.get_param("~marker_line_width", 0.03)
        self.marker_alpha = rospy.get_param("~marker_alpha", 1.0)

        self.queue_maxlen = rospy.get_param("~queue_maxlen", 1000)
        self.max_process_per_tick = rospy.get_param("~max_process_per_tick", 200)

        # -------------------------
        # delta-state specific params
        # -------------------------
        self.control_alpha = rospy.get_param("~control_alpha", 0.99)
        self.use_back_sync = rospy.get_param("~use_back_sync", True)
        self.back_sync_max_age = rospy.get_param("~back_sync_max_age", 0.9)
        self.back_sync_r_scale = rospy.get_param("~back_sync_r_scale", 1.5)
        self.back_sync_age_scale = rospy.get_param("~back_sync_age_scale", 1.0)
        self.max_pair_dt = rospy.get_param("~max_pair_dt", 2.0)

        self.delta_linear_deadband = rospy.get_param("~delta_linear_deadband", 0.0)
        self.delta_angular_deadband = rospy.get_param("~delta_angular_deadband", 0.0)

        cov_min_eig = rospy.get_param("~cov_min_eig", 1e-9)

        # -------------------------
        # noise params
        # -------------------------
        # 状态过程噪声：Delta 与 Delta_dot
        q_delta_pos = rospy.get_param("~q_delta_pos", rospy.get_param("~q_pos", 0.02))
        q_delta_ang = rospy.get_param("~q_delta_ang", rospy.get_param("~q_ang", 0.02))
        q_rate_pos = rospy.get_param("~q_rate_pos", rospy.get_param("~q_vel", 0.10))
        q_rate_ang = rospy.get_param("~q_rate_ang", rospy.get_param("~q_yaw_rate", 0.05))

        # 观测噪声：front/back 的相对变换 Delta
        r_front_delta_pos = rospy.get_param("~r_front_delta_pos", rospy.get_param("~r_front_pos", 0.20))
        r_front_delta_ang = rospy.get_param("~r_front_delta_ang", rospy.get_param("~r_front_ang", 0.10))
        r_back_delta_pos = rospy.get_param("~r_back_delta_pos", rospy.get_param("~r_back_pos", 0.08))
        r_back_delta_ang = rospy.get_param("~r_back_delta_ang", rospy.get_param("~r_back_ang", 0.05))

        # 局部增量状态初值协方差
        init_delta_pos_std = rospy.get_param("~init_delta_pos_std", 0.05)
        init_delta_ang_std = rospy.get_param("~init_delta_ang_std", 0.05)
        init_rate_pos_std = rospy.get_param("~init_rate_pos_std", 0.50)
        init_rate_ang_std = rospy.get_param("~init_rate_ang_std", 0.30)

        # 每次把增量 Delta 提交到全局 pose 之后，需要把本地 Delta 状态清零重新开始
        reset_delta_pos_std = rospy.get_param("~reset_delta_pos_std", 0.01)
        reset_delta_ang_std = rospy.get_param("~reset_delta_ang_std", 0.01)
        reset_rate_pos_inflate_std = rospy.get_param("~reset_rate_pos_inflate_std", 0.02)
        reset_rate_ang_inflate_std = rospy.get_param("~reset_rate_ang_inflate_std", 0.02)

        # 全局 pose 初始协方差（第一次用 front 初始化世界 pose 时）
        init_global_pos_std = rospy.get_param("~init_global_pos_std", 0.50)
        init_global_ang_std = rospy.get_param("~init_global_ang_std", 0.30)

        Q = np.diag([
            q_delta_pos ** 2, q_delta_pos ** 2, q_delta_pos ** 2,
            q_delta_ang ** 2, q_delta_ang ** 2, q_delta_ang ** 2,
            q_rate_pos ** 2, q_rate_pos ** 2, q_rate_pos ** 2,
            q_rate_ang ** 2, q_rate_ang ** 2, q_rate_ang ** 2,
        ])

        R_front = np.diag([
            r_front_delta_pos ** 2, r_front_delta_pos ** 2, r_front_delta_pos ** 2,
            r_front_delta_ang ** 2, r_front_delta_ang ** 2, r_front_delta_ang ** 2,
        ])

        R_back = np.diag([
            r_back_delta_pos ** 2, r_back_delta_pos ** 2, r_back_delta_pos ** 2,
            r_back_delta_ang ** 2, r_back_delta_ang ** 2, r_back_delta_ang ** 2,
        ])

        self.init_local_P = np.diag([
            init_delta_pos_std ** 2, init_delta_pos_std ** 2, init_delta_pos_std ** 2,
            init_delta_ang_std ** 2, init_delta_ang_std ** 2, init_delta_ang_std ** 2,
            init_rate_pos_std ** 2, init_rate_pos_std ** 2, init_rate_pos_std ** 2,
            init_rate_ang_std ** 2, init_rate_ang_std ** 2, init_rate_ang_std ** 2,
        ])

        self.reset_delta_cov = np.diag([
            reset_delta_pos_std ** 2, reset_delta_pos_std ** 2, reset_delta_pos_std ** 2,
            reset_delta_ang_std ** 2, reset_delta_ang_std ** 2, reset_delta_ang_std ** 2,
        ])

        self.reset_rate_inflate_cov = np.diag([
            reset_rate_pos_inflate_std ** 2, reset_rate_pos_inflate_std ** 2, reset_rate_pos_inflate_std ** 2,
            reset_rate_ang_inflate_std ** 2, reset_rate_ang_inflate_std ** 2, reset_rate_ang_inflate_std ** 2,
        ])

        self.init_global_pose_cov = np.diag([
            init_global_pos_std ** 2, init_global_pos_std ** 2, init_global_pos_std ** 2,
            init_global_ang_std ** 2, init_global_ang_std ** 2, init_global_ang_std ** 2,
        ])

        self.kf = DeltaStateKalmanFilter(
            Q=Q,
            R_front=R_front,
            R_back=R_back,
            control_alpha=self.control_alpha,
            cov_min_eig=cov_min_eig,
        )

        # -------------------------
        # runtime state
        # -------------------------
        self.lock = threading.Lock()

        self.front_queue = deque(maxlen=self.queue_maxlen)
        self.back_queue = deque(maxlen=self.queue_maxlen)

        self.last_front_raw_meas = None
        self.last_back_raw_meas = None

        # pending back: 只缓存一条最新、未消费的 back 增量导数
        self.pending_back_meas = None
        self.pending_back_delta_rate = None
        self.last_back_consumed_stamp = None

        self.initialized = False
        self.filter_time = None
        self.last_pub_time = None

        # 局部增量滤波器状态（每个 front step 提交后会 reset）
        self.x = np.zeros(12, dtype=float)
        self.P = np.array(self.init_local_P, dtype=float)

        # 全局输出 pose：由滤波后的 Delta 逐步迭代出来
        self.global_position = np.zeros(3, dtype=float)
        self.global_orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.global_pose_cov = np.array(self.init_global_pose_cov, dtype=float)

        # -------------------------
        # pubs / subs
        # -------------------------
        self.pub_odom = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.pub_marker = rospy.Publisher(self.marker_topic, Marker, queue_size=10)

        self.pub_future_global_odom = rospy.Publisher(self.future_global_output_topic, Odometry, queue_size=10)
        self.pub_future_local_odom = rospy.Publisher(self.future_local_output_topic, Odometry, queue_size=10)

        self.pub_future_global_marker = rospy.Publisher(self.future_global_marker_topic, Marker, queue_size=10)
        self.pub_future_local_marker = rospy.Publisher(self.future_local_marker_topic, Marker, queue_size=10)

        rospy.Subscriber(self.front_topic, Odometry, self.front_cb, queue_size=100)
        rospy.Subscriber(self.back_topic, Odometry, self.back_cb, queue_size=100)

        # -------------------------
        # timers
        # -------------------------
        self.fusion_timer = rospy.Timer(
            rospy.Duration(1.0 / self.fusion_rate),
            self.fusion_timer_cb,
        )
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.publish_timer_cb,
        )

    # --------------------------------------------------------
    # callbacks: only enqueue
    # --------------------------------------------------------

    def front_cb(self, msg):
        meas = odom_msg_to_measurement(msg, "front")
        with self.lock:
            self.front_queue.append(meas)

    def back_cb(self, msg):
        meas = odom_msg_to_measurement(msg, "back")
        with self.lock:
            self.back_queue.append(meas)

    # --------------------------------------------------------
    # queue helpers
    # --------------------------------------------------------

    def _peek_next_source(self):
        if len(self.front_queue) == 0 and len(self.back_queue) == 0:
            return None
        if len(self.front_queue) == 0:
            return "back"
        if len(self.back_queue) == 0:
            return "front"
        if self.front_queue[0].timestamp <= self.back_queue[0].timestamp:
            return "front"
        return "back"

    def _pop_next_measurement(self):
        src = self._peek_next_source()
        if src is None:
            return None
        if src == "front":
            return self.front_queue.popleft()
        return self.back_queue.popleft()

    # --------------------------------------------------------
    # increment helpers
    # --------------------------------------------------------

    def _build_delta_with_checks(self, prev_meas, curr_meas, source):
        if prev_meas is None or curr_meas is None:
            return None, None

        dt = curr_meas.timestamp - prev_meas.timestamp
        if dt <= 1e-6:
            return None, None

        if dt > self.max_pair_dt:
            rospy.logwarn_throttle(
                1.0,
                "Skip %s pair because dt=%.6f > max_pair_dt=%.6f",
                source,
                dt,
                self.max_pair_dt,
            )
            return None, None

        delta = compute_relative_delta(
            prev_meas.position,
            prev_meas.orientation,
            curr_meas.position,
            curr_meas.orientation,
        )
        delta = apply_deadband_delta(
            delta,
            linear_deadband=self.delta_linear_deadband,
            angular_deadband=self.delta_angular_deadband,
        )
        return delta, dt

    def _clear_pending_back(self, consumed_stamp=None):
        if consumed_stamp is not None:
            self.last_back_consumed_stamp = consumed_stamp
        self.pending_back_meas = None
        self.pending_back_delta_rate = None

    # --------------------------------------------------------
    # init / commit / publish-predict
    # --------------------------------------------------------

    def _init_from_front_measurement(self, meas):
        self.initialized = True
        self.filter_time = meas.timestamp

        self.global_position = meas.position.copy()
        self.global_orientation = meas.orientation.copy()
        self.global_pose_cov = np.array(self.init_global_pose_cov, dtype=float)

        self.x = np.zeros(12, dtype=float)
        self.P = np.array(self.init_local_P, dtype=float)

        self.last_front_raw_meas = meas
        rospy.loginfo("Delta-state filter initialized from front at t=%.6f", meas.timestamp)

    def _commit_fused_delta(self):
        """
        把当前局部滤波后的 Delta 提交到全局 pose，
        然后把局部 Delta 清零，保留 Delta_dot 进入下一小段。
        """
        delta_fused = self.x[0:6].copy()
        rate_fused = self.x[6:12].copy()

        self.global_position, self.global_orientation = apply_local_delta_to_pose(
            self.global_position,
            self.global_orientation,
            delta_fused,
        )

        self.global_pose_cov = self.kf.project_cov(self.global_pose_cov + self.P[0:6, 0:6])

        P_rate = self.kf.project_cov(self.P[6:12, 6:12] + self.reset_rate_inflate_cov)

        x_next = np.zeros(12, dtype=float)
        x_next[6:12] = rate_fused

        P_next = np.zeros((12, 12), dtype=float)
        P_next[0:6, 0:6] = self.reset_delta_cov
        P_next[6:12, 6:12] = P_rate
        P_next = self.kf.project_cov(P_next)

        self.x = x_next
        self.P = P_next

    def _predict_publish_pose_to_now(self, now_sec):
        return self._predict_pose_from_reference(
            self.global_position,
            self.global_orientation,
            self.global_pose_cov,
            self.x,
            self.P,
            max(float(now_sec - self.filter_time), 0.0),
        )

    def _predict_pose_from_reference(self, ref_pos, ref_ori, ref_pose_cov, x_ref, P_ref, dt_future):
        dt_future = max(float(dt_future), 0.0)
        if dt_future <= 1e-9:
            return (
                np.asarray(ref_pos, dtype=float).copy(),
                np.asarray(ref_ori, dtype=float).copy(),
                np.asarray(ref_pose_cov, dtype=float).copy(),
                np.asarray(x_ref, dtype=float).copy(),
                np.asarray(P_ref, dtype=float).copy(),
            )

        # 只做发布临时预测，不污染内部状态
        x_pub, P_pub = self.kf.predict(x_ref, P_ref, dt_future, u=None)
        delta_pub = x_pub[0:6].copy()

        pos_pub, ori_pub = apply_local_delta_to_pose(
            ref_pos,
            ref_ori,
            delta_pub,
        )
        pose_cov_pub = self.kf.project_cov(np.asarray(ref_pose_cov, dtype=float) + P_pub[0:6, 0:6])

        return pos_pub, ori_pub, pose_cov_pub, x_pub, P_pub

    def _solve_future_dt_by_distance(self, ref_pos, ref_ori, x_ref, P_ref):
        dist_target = max(float(self.publish_predict_distance), 0.0)
        if dist_target <= 1e-9:
            return 0.0

        step = max(float(self.publish_predict_step_sec), 1e-3)
        max_sec = max(float(self.publish_predict_max_sec), step)

        x_tmp = np.asarray(x_ref, dtype=float).copy()
        P_tmp = np.asarray(P_ref, dtype=float).copy()
        dt_acc = 0.0

        while dt_acc < max_sec - 1e-12:
            dt_step = min(step, max_sec - dt_acc)
            x_tmp, P_tmp = self.kf.predict(x_tmp, P_tmp, dt_step, u=None)
            dt_acc += dt_step

            delta_tmp = x_tmp[0:6].copy()
            pos_tmp, _ = apply_local_delta_to_pose(ref_pos, ref_ori, delta_tmp)
            dist_now = np.linalg.norm(np.asarray(pos_tmp, dtype=float) - np.asarray(ref_pos, dtype=float))
            if dist_now >= dist_target:
                return dt_acc

        rospy.logwarn_throttle(
            1.0,
            "Future distance target %.6f m not reached within publish_predict_max_sec=%.6f s",
            dist_target,
            max_sec,
        )
        return max_sec

    def _build_future_prediction_bundle(self, ref_pos, ref_ori, ref_pose_cov, x_ref, P_ref):
        extra_sec = max(float(self.publish_predict_extra_sec), 0.0)
        dist_sec = self._solve_future_dt_by_distance(ref_pos, ref_ori, x_ref, P_ref)
        dt_future = max(extra_sec, dist_sec)

        target_pos, target_ori, target_pose_cov, x_future, P_future = self._predict_pose_from_reference(
            ref_pos,
            ref_ori,
            ref_pose_cov,
            x_ref,
            P_ref,
            dt_future,
        )

        local_delta = compute_relative_delta(ref_pos, ref_ori, target_pos, target_ori)
        local_pos = local_delta[0:3].copy()
        local_ori = ypr_to_quat(local_delta[3], local_delta[4], local_delta[5])
        local_pose_cov = self.kf.project_cov(P_future[0:6, 0:6])

        return {
            "dt_future": dt_future,
            "target_pos": target_pos,
            "target_ori": target_ori,
            "target_pose_cov": target_pose_cov,
            "x_future": x_future,
            "P_future": P_future,
            "local_pos": local_pos,
            "local_ori": local_ori,
            "local_pose_cov": local_pose_cov,
        }

    # --------------------------------------------------------
    # back handling
    # --------------------------------------------------------

    def _cache_back_measurement(self, meas):
        prev_back = self.last_back_raw_meas

        if prev_back is not None and meas.timestamp <= prev_back.timestamp + 1e-9:
            rospy.logwarn_throttle(
                1.0,
                "Ignore non-increasing back measurement: %.6f <= %.6f",
                meas.timestamp,
                prev_back.timestamp,
            )
            return

        delta_back, dt_back = self._build_delta_with_checks(prev_back, meas, "back")
        self.last_back_raw_meas = meas

        if delta_back is None or dt_back is None:
            return

        delta_rate_back = delta_to_rate(delta_back, dt_back)

        if self.last_back_consumed_stamp is not None and meas.timestamp <= self.last_back_consumed_stamp + 1e-9:
            return

        if self.pending_back_meas is None or meas.timestamp >= self.pending_back_meas.timestamp - 1e-9:
            self.pending_back_meas = meas
            self.pending_back_delta_rate = delta_rate_back

    def _maybe_update_with_pending_back(self, dt_front, current_front_time):
        if not self.use_back_sync:
            return

        if self.pending_back_meas is None or self.pending_back_delta_rate is None:
            return

        if self.pending_back_meas.timestamp > current_front_time + 1e-9:
            return

        age = current_front_time - self.pending_back_meas.timestamp
        if age < -1e-9:
            return

        if age > self.back_sync_max_age:
            rospy.logwarn_throttle(
                1.0,
                "Drop stale pending back increment: age=%.6f > max_age=%.6f",
                age,
                self.back_sync_max_age,
            )
            self._clear_pending_back(consumed_stamp=self.pending_back_meas.timestamp)
            return

        age_scale = 1.0 + self.back_sync_age_scale * age
        if age_scale < 1.0:
            age_scale = 1.0

        # 用 back 的 Delta_dot 同步到“当前 front 这一小段”的 Delta 观测
        z_back_sync = self.pending_back_delta_rate * dt_front
        z_back_sync = wrap_vec_angles(z_back_sync, [3, 4, 5])

        R_back_sync = self.kf.project_cov(
            self.kf.R_back * float(self.back_sync_r_scale) * float(age_scale)
        )

        self.x, self.P = self.kf.update_delta(self.x, self.P, z_back_sync, R_back_sync)
        #self._clear_pending_back(consumed_stamp=self.pending_back_meas.timestamp)

    # --------------------------------------------------------
    # fusion logic
    # --------------------------------------------------------

    def _process_front_measurement(self, meas):
        if self.last_front_raw_meas is not None and meas.timestamp <= self.last_front_raw_meas.timestamp + 1e-9:
            rospy.logwarn_throttle(
                1.0,
                "Ignore non-increasing front measurement: %.6f <= %.6f",
                meas.timestamp,
                self.last_front_raw_meas.timestamp,
            )
            return

        delta_front, dt_front = self._build_delta_with_checks(self.last_front_raw_meas, meas, "front")
        if delta_front is None or dt_front is None:
            self.last_front_raw_meas = meas
            return

        # 控制输入：变化量的导数（Delta_dot）
        u_front = delta_to_rate(delta_front, dt_front)

        # 先用控制输入预测局部增量状态
        self.x, self.P = self.kf.predict(self.x, self.P, dt_front, u=u_front)

        # 再用 front 的“相对变换 Delta”做观测更新
        self.x, self.P = self.kf.update_delta(self.x, self.P, delta_front, self.kf.R_front)

        # 如果有历史 back，则把 back 的 Delta_dot 同步成当前 front interval 的 Delta 再更新一次
        self._maybe_update_with_pending_back(dt_front, meas.timestamp)

        # 把这一小段融合后的 Delta 提交到全局 pose
        self._commit_fused_delta()

        self.filter_time = meas.timestamp
        self.last_front_raw_meas = meas

    def _process_back_measurement(self, meas):
        self._cache_back_measurement(meas)

    def fusion_timer_cb(self, _event):
        with self.lock:
            processed = 0
            while processed < self.max_process_per_tick:
                meas = self._pop_next_measurement()
                if meas is None:
                    break

                if not self.initialized:
                    if meas.source == "front":
                        self._init_from_front_measurement(meas)
                    else:
                        # 初始化前到达的 back 仅用于建立 back 历史，不单独启动全局输出
                        self._process_back_measurement(meas)
                    processed += 1
                    continue

                if meas.source == "front":
                    self._process_front_measurement(meas)
                else:
                    self._process_back_measurement(meas)

                processed += 1

    # --------------------------------------------------------
    # marker
    # --------------------------------------------------------

    def build_frame_marker(self, stamp, px, py, pz, qx, qy, qz, qw, frame_id=None, ns="fused_pose_frame", marker_id=0):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame if frame_id is None else frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.pose.position.x = px
        marker.pose.position.y = py
        marker.pose.position.z = pz
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = self.marker_line_width

        L = self.marker_scale
        origin = Point(0.0, 0.0, 0.0)
        x_axis = Point(L, 0.0, 0.0)
        y_axis = Point(0.0, L, 0.0)
        z_axis = Point(0.0, 0.0, L)

        marker.points = [
            origin, x_axis,
            origin, y_axis,
            origin, z_axis,
        ]

        red = ColorRGBA(1.0, 0.0, 0.0, self.marker_alpha)
        green = ColorRGBA(0.0, 1.0, 0.0, self.marker_alpha)
        blue = ColorRGBA(0.0, 0.0, 1.0, self.marker_alpha)

        marker.colors = [
            red, red,
            green, green,
            blue, blue,
        ]

        marker.lifetime = rospy.Duration(0.0)
        marker.frame_locked = False
        return marker

    def build_local_target_line_marker(self, stamp, px, py, pz, frame_id):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "future_local_target_line"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_line_width

        marker.points = [
            Point(0.0, 0.0, 0.0),
            Point(px, py, pz),
        ]

        line_color = ColorRGBA(1.0, 1.0, 0.0, self.marker_alpha)
        marker.colors = [line_color, line_color]
        marker.lifetime = rospy.Duration(0.0)
        marker.frame_locked = False
        return marker

    # --------------------------------------------------------
    # publish timer
    # --------------------------------------------------------

    def publish_timer_cb(self, _event):
        with self.lock:
            if not self.initialized:
                return

            now_time = rospy.Time.now()
            now_sec = now_time.to_sec()

            # /fused_odom 只预测到现在
            if self.publish_predict_to_now and now_sec > self.filter_time + 1e-9:
                ref_pos, ref_ori, ref_pose_cov, x_ref, P_ref = self._predict_publish_pose_to_now(now_sec)
                ref_time = now_sec
            else:
                ref_pos = self.global_position.copy()
                ref_ori = self.global_orientation.copy()
                ref_pose_cov = self.global_pose_cov.copy()
                x_ref = self.x.copy()
                P_ref = self.P.copy()
                ref_time = float(self.filter_time)

            current_stamp = now_time if self.publish_use_now_stamp else rospy.Time.from_sec(ref_time)

            if self.last_pub_time is not None:
                if abs(current_stamp.to_sec() - self.last_pub_time.to_sec()) < 1e-9:
                    return

            # 当前 odom: 仍然只到 now
            msg = Odometry()
            msg.header.stamp = current_stamp
            msg.header.frame_id = self.world_frame
            msg.child_frame_id = self.child_frame

            cpx, cpy, cpz = ref_pos[0], ref_pos[1], ref_pos[2]
            cqx, cqy, cqz, cqw = ref_ori[0], ref_ori[1], ref_ori[2], ref_ori[3]

            current_rate = x_ref[6:12].copy()
            cvx_local, cvy_local, cvz_local = current_rate[0], current_rate[1], current_rate[2]
            cyaw_rate, cpitch_rate, croll_rate = current_rate[3], current_rate[4], current_rate[5]

            msg.pose.pose.position.x = cpx
            msg.pose.pose.position.y = cpy
            msg.pose.pose.position.z = cpz
            msg.pose.pose.orientation = Quaternion(x=cqx, y=cqy, z=cqz, w=cqw)

            msg.twist.twist.linear.x = cvx_local
            msg.twist.twist.linear.y = cvy_local
            msg.twist.twist.linear.z = cvz_local
            msg.twist.twist.angular.x = croll_rate
            msg.twist.twist.angular.y = cpitch_rate
            msg.twist.twist.angular.z = cyaw_rate

            pose_cov = np.zeros((6, 6), dtype=float)
            pose_idx = [0, 1, 2, 5, 4, 3]
            for i in range(6):
                for j in range(6):
                    pose_cov[i, j] = ref_pose_cov[pose_idx[i], pose_idx[j]]
            msg.pose.covariance = pose_cov.reshape(-1).tolist()

            twist_cov = np.zeros((6, 6), dtype=float)
            current_rate_cov = self.kf.project_cov(P_ref[6:12, 6:12])
            twist_idx = [0, 1, 2, 5, 4, 3]
            for i in range(6):
                for j in range(6):
                    twist_cov[i, j] = current_rate_cov[twist_idx[i], twist_idx[j]]
            msg.twist.covariance = twist_cov.reshape(-1).tolist()

            self.pub_odom.publish(msg)

            current_marker = self.build_frame_marker(
                stamp=current_stamp,
                px=cpx,
                py=cpy,
                pz=cpz,
                qx=cqx,
                qy=cqy,
                qz=cqz,
                qw=cqw,
                frame_id=self.world_frame,
                ns="fused_pose_frame",
                marker_id=0,
            )
            self.pub_marker.publish(current_marker)

            # 未来预测：单独发布 future global / future local
            future_bundle = self._build_future_prediction_bundle(ref_pos, ref_ori, ref_pose_cov, x_ref, P_ref)
            future_dt = future_bundle["dt_future"]

            if future_dt > 1e-9:
                future_stamp = now_time if self.publish_use_now_stamp else rospy.Time.from_sec(ref_time + future_dt)

                future_pos = future_bundle["target_pos"]
                future_ori = future_bundle["target_ori"]
                future_pose_cov = future_bundle["target_pose_cov"]
                x_future = future_bundle["x_future"]
                P_future = future_bundle["P_future"]

                future_rate = x_future[6:12].copy()
                fvx_local, fvy_local, fvz_local = future_rate[0], future_rate[1], future_rate[2]
                fyaw_rate, fpitch_rate, froll_rate = future_rate[3], future_rate[4], future_rate[5]

                future_twist_cov = np.zeros((6, 6), dtype=float)
                future_rate_cov = self.kf.project_cov(P_future[6:12, 6:12])
                for i in range(6):
                    for j in range(6):
                        future_twist_cov[i, j] = future_rate_cov[twist_idx[i], twist_idx[j]]

                if self.publish_future_global_odom and self.pub_future_global_odom is not None:
                    future_global_msg = Odometry()
                    future_global_msg.header.stamp = future_stamp
                    future_global_msg.header.frame_id = self.world_frame
                    future_global_msg.child_frame_id = self.child_frame + "_future_global"

                    future_global_msg.pose.pose.position.x = future_pos[0]
                    future_global_msg.pose.pose.position.y = future_pos[1]
                    future_global_msg.pose.pose.position.z = future_pos[2]
                    future_global_msg.pose.pose.orientation = Quaternion(
                        x=future_ori[0], y=future_ori[1], z=future_ori[2], w=future_ori[3]
                    )

                    future_global_msg.twist.twist.linear.x = fvx_local
                    future_global_msg.twist.twist.linear.y = fvy_local
                    future_global_msg.twist.twist.linear.z = fvz_local
                    future_global_msg.twist.twist.angular.x = froll_rate
                    future_global_msg.twist.twist.angular.y = fpitch_rate
                    future_global_msg.twist.twist.angular.z = fyaw_rate

                    future_pose_cov_ros = np.zeros((6, 6), dtype=float)
                    for i in range(6):
                        for j in range(6):
                            future_pose_cov_ros[i, j] = future_pose_cov[pose_idx[i], pose_idx[j]]
                    future_global_msg.pose.covariance = future_pose_cov_ros.reshape(-1).tolist()
                    future_global_msg.twist.covariance = future_twist_cov.reshape(-1).tolist()
                    self.pub_future_global_odom.publish(future_global_msg)

                    future_global_marker = self.build_frame_marker(
                        stamp=future_stamp,
                        px=future_pos[0],
                        py=future_pos[1],
                        pz=future_pos[2],
                        qx=future_ori[0],
                        qy=future_ori[1],
                        qz=future_ori[2],
                        qw=future_ori[3],
                        frame_id=self.world_frame,
                        ns="future_global_pose_frame",
                        marker_id=0,
                    )
                    self.pub_future_global_marker.publish(future_global_marker)

                if self.publish_future_local_odom and self.pub_future_local_odom is not None:
                    future_local_msg = Odometry()
                    future_local_msg.header.stamp = future_stamp
                    future_local_msg.header.frame_id = self.future_local_frame
                    future_local_msg.child_frame_id = self.future_local_child_frame

                    lpx, lpy, lpz = future_bundle["local_pos"]
                    lqx, lqy, lqz, lqw = future_bundle["local_ori"]

                    future_local_msg.pose.pose.position.x = lpx
                    future_local_msg.pose.pose.position.y = lpy
                    future_local_msg.pose.pose.position.z = lpz
                    future_local_msg.pose.pose.orientation = Quaternion(x=lqx, y=lqy, z=lqz, w=lqw)

                    future_local_msg.twist.twist.linear.x = fvx_local
                    future_local_msg.twist.twist.linear.y = fvy_local
                    future_local_msg.twist.twist.linear.z = fvz_local
                    future_local_msg.twist.twist.angular.x = froll_rate
                    future_local_msg.twist.twist.angular.y = fpitch_rate
                    future_local_msg.twist.twist.angular.z = fyaw_rate

                    local_pose_cov_ros = np.zeros((6, 6), dtype=float)
                    for i in range(6):
                        for j in range(6):
                            local_pose_cov_ros[i, j] = future_bundle["local_pose_cov"][pose_idx[i], pose_idx[j]]
                    future_local_msg.pose.covariance = local_pose_cov_ros.reshape(-1).tolist()
                    future_local_msg.twist.covariance = future_twist_cov.reshape(-1).tolist()
                    self.pub_future_local_odom.publish(future_local_msg)

                    future_local_axes_marker = self.build_frame_marker(
                        stamp=future_stamp,
                        px=lpx,
                        py=lpy,
                        pz=lpz,
                        qx=lqx,
                        qy=lqy,
                        qz=lqz,
                        qw=lqw,
                        frame_id=self.future_local_frame,
                        ns="future_local_pose_frame",
                        marker_id=0,
                    )
                    self.pub_future_local_marker.publish(future_local_axes_marker)

                    future_local_line_marker = self.build_local_target_line_marker(
                        stamp=future_stamp,
                        px=lpx,
                        py=lpy,
                        pz=lpz,
                        frame_id=self.future_local_frame,
                    )
                    self.pub_future_local_marker.publish(future_local_line_marker)

            self.last_pub_time = current_stamp

    def spin(self):
        rospy.spin()


# ============================================================
# main
# ============================================================

def main():
    # 为了兼容你现有 launch，不改 node name
    rospy.init_node("dual_odom_ukf_node")
    node = DualOdomDeltaStateNode()
    node.spin()


if __name__ == "__main__":
    main()
