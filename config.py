import math


TAG_GATE_UDS_PATH = "/tmp/qzj_apriltag_gate.sock"
TAG_GATE_LINEAR_LIMIT = 0.05
TAG_GATE_ANGULAR_LIMIT = math.radians(5.0)
TAG_GATE_HEARTBEAT_S = 0.1

vx_lim = 100 # x 方向运动速度的限制
vy_lim = 100 # y 方向运动速度的限制
ax_lim = 100 # x 方向运动加速度的限制
ay_lim = 100 # y 方向运动加速度的限制

v_yaw_lim = 100 # 底盘旋转速度限制 deg/s
a_yaw_lim = 100 # 底盘旋转加速度限制 deg/s2

vh_lim = 100 # 五连杆上下移动限速
ah_lim = 100 # 五连杆上下移动限加速度

# 连杆两电机的限制
dq_lim = 100
ddq_lim = 100
