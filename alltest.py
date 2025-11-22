import numpy as np

# # 问题一求解
# v_U = 120       # m/s
# P_U0 = np.array([17800, 0, 1800])
# t_release = 1.5        # 修改: 投放时间, 原 t_release
# t_burst = 5.1         # 修改: 起爆时间, 直接给定
# angle_deg_U = 180                                           # 新增: 无人机飞行方向角度(与x轴正向夹角)



# 问题二验证
v_U = 133.120      # m/s，更新后FY1飞行速度（原数据已替换）
P_U0 = np.array([17800, 0, 1800])
t_release = 0.284      # s，更新后烟幕弹投放时刻（对应原 t_drop）
t_burst = 0.526+0.284          # s，更新后烟幕弹起爆时刻（对应原 t_det）
angle_deg_U =6.327    # 度，更新后与x轴正向夹角（航向角）



# # 问题二验证
# v_U = 71.664       # m/s，更新后FY1飞行速度（原数据已替换）
# P_U0 = np.array([17800, 0, 1800])
# t_release = 0.122        # s，更新后烟幕弹投放时刻（对应原 t_drop）
# t_burst = 2.639          # s，更新后烟幕弹起爆时刻（对应原 t_det）
# angle_deg_U = 177.321    # 度，更新后与x轴正向夹角（航向角）

# # 问题二验证
# v_U = 75.731       # m/s，更新后FY1飞行速度（原数据已替换）
# P_U0 = np.array([17800, 0, 1800])
# t_release = 0.181     # s，更新后烟幕弹投放时刻（对应原 t_drop）
# t_burst = 2.807        # s，更新后烟幕弹起爆时刻（对应原 t_det）
# angle_deg_U = 177.598    # 度，更新后与x轴正向夹角（航向角）

# # 问题三验证
# # 无人机固定飞行参数
# v_U = 119.39       # m/s，无人机飞行速度
# angle_deg_U = 179.20  # 度，无人机飞行方位角
# P_U0 = np.array([17800, 0, 1800])  # 无人机初始位置（沿用历史基准坐标）
# # 烟幕弹1专属参数
# t_release = 0.102    # s，投放时间
# flight_time = 3.626  # s，飞行时间（从投放至起爆的时间）
# t_burst = 3.728      # s，起爆时间（与投放时间+飞行时间一致：0.102+3.626=3.728）
# # 烟幕弹2专属参数
# t_release = 1.951    # s，投放时间
# flight_time = 3.991  # s，飞行时间（从投放至起爆的时间）
# t_burst = 5.942      # s，起爆时间（与投放时间+飞行时间一致：1.951+3.991=5.942）
# # 烟幕弹3专属参数
# t_release = 4.992    # s，投放时间
# flight_time = 0.630  # s，飞行时间（从投放至起爆的时间）
# t_burst = 5.622      # s，起爆时间（与投放时间+飞行时间一致：4.992+0.630=5.622）



# # 问题四验证
# #fy1
# v_U = 82.01       # m/s，飞行速度（详细投放策略中FY1的飞行速度）
# P_U0 = np.array([17800, 0, 1800])
# t_release = 0.11        # 投放时间, 对应原 t_drop
# t_burst = 2.61          # 起爆时间, 对应原 t_det
# angle_deg_U = 178.62                                           # 单位：度
# #fy2
# v_U = 123.39      # m/s，飞行速度（详细投放策略中FY2的飞行速度）
# P_U0 = np.array([12000, 1400, 1400])
# t_release = 7.10        # 投放时间, 对应原 t_drop
# t_burst = 15.21         # 起爆时间, 对应原 t_det
# angle_deg_U = 225.87                                           # 单位：度
# #fy3
# v_U = 128.16      # m/s，飞行速度（详细投放策略中FY3的飞行速度）
# P_U0 = np.array([6000, -3000, 700])
# t_release = 20.06       # 投放时间, 对应原 t_drop
# t_burst = 27.07         # 起爆时间, 对应原 t_det
# angle_deg_U = 115.97                                           # 单位：度




# 位置向量（单位：m）
P_M0 = np.array([20000.0, 0.0, 2000.0])    # 导弹初始
# #问题五验证
# v_U = 123.82
# P_U0 = np.array([6000,-3000,700])
# t_release = 19.86
# t_burst = 23.26
# angle_deg_U = 1.559
# P_M0 = np.array([18000,-600,1900])
# config_id = 111  # 三位数编号：百位=无人机(FY1-5)，十位=烟幕弹(1-3)，个位=导弹(M1-3)

# # FY1系列（百位=1）
# if config_id == 111:  # FY1-弹1-M1
#     v_U = 70.00
#     P_U0 = np.array([17800, 0, 1800])
#     t_release = 0.536
#     t_burst = 1.540
#     angle_deg_U = 11.06
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 112:  # FY1-弹1-M2
#     v_U = 70.00
#     P_U0 = np.array([17800, 0, 1800])
#     t_release = 0.536
#     t_burst = 1.540
#     angle_deg_U = 11.06
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 113:  # FY1-弹1-M3
#     v_U = 70.00
#     P_U0 = np.array([17800, 0, 1800])
#     t_release = 0.536
#     t_burst = 1.540
#     angle_deg_U = 11.06
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 121:  # FY1-弹2-M1
#     v_U = 70.00
#     P_U0 = np.array([17800, 0, 1800])
#     t_release = 4.322
#     t_burst = 6.401
#     angle_deg_U = 11.06
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 122:  # FY1-弹2-M2
#     v_U = 70.00
#     P_U0 = np.array([17800, 0, 1800])
#     t_release = 4.322
#     t_burst = 6.401
#     angle_deg_U = 11.06
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 123:  # FY1-弹2-M3
#     v_U = 70.00
#     P_U0 = np.array([17800, 0, 1800])
#     t_release = 4.322
#     t_burst = 6.401
#     angle_deg_U = 11.06
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 131:  # FY1-弹3-M1
#     v_U = 70.00
#     P_U0 = np.array([17800, 0, 1800])
#     t_release = 5.522
#     t_burst = 9.207
#     angle_deg_U = 11.06
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 132:  # FY1-弹3-M2
#     v_U = 70.00
#     P_U0 = np.array([17800, 0, 1800])
#     t_release = 5.522
#     t_burst = 9.207
#     angle_deg_U = 11.06
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 133:  # FY1-弹3-M3
#     v_U = 70.00
#     P_U0 = np.array([17800, 0, 1800])
#     t_release = 5.522
#     t_burst = 9.207
#     angle_deg_U = 11.06
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# # FY2系列（百位=2）
# elif config_id == 211:  # FY2-弹1-M1
#     v_U = 74.36
#     P_U0 = np.array([12000, 1400, 1400])
#     t_release = 3.675
#     t_burst = 9.019
#     angle_deg_U = 65.93
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 212:  # FY2-弹1-M2
#     v_U = 74.36
#     P_U0 = np.array([12000, 1400, 1400])
#     t_release = 3.675
#     t_burst = 9.019
#     angle_deg_U = 65.93
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 213:  # FY2-弹1-M3
#     v_U = 74.36
#     P_U0 = np.array([12000, 1400, 1400])
#     t_release = 3.675
#     t_burst = 9.019
#     angle_deg_U = 65.93
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 221:  # FY2-弹2-M1
#     v_U = 74.36
#     P_U0 = np.array([12000, 1400, 1400])
#     t_release = 5.772
#     t_burst = 11.001
#     angle_deg_U = 65.93
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 222:  # FY2-弹2-M2
#     v_U = 74.36
#     P_U0 = np.array([12000, 1400, 1400])
#     t_release = 5.772
#     t_burst = 11.001
#     angle_deg_U = 65.93
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 223:  # FY2-弹2-M3
#     v_U = 74.36
#     P_U0 = np.array([12000, 1400, 1400])
#     t_release = 5.772
#     t_burst = 11.001
#     angle_deg_U = 65.93
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 231:  # FY2-弹3-M1
#     v_U = 74.36
#     P_U0 = np.array([12000, 1400, 1400])
#     t_release = 7.229
#     t_burst = 10.115
#     angle_deg_U = 65.93
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 232:  # FY2-弹3-M2
#     v_U = 74.36
#     P_U0 = np.array([12000, 1400, 1400])
#     t_release = 7.229
#     t_burst = 10.115
#     angle_deg_U = 65.93
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 233:  # FY2-弹3-M3
#     v_U = 74.36
#     P_U0 = np.array([12000, 1400, 1400])
#     t_release = 7.229
#     t_burst = 10.115
#     angle_deg_U = 65.93
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# # FY3系列（百位=3）
# elif config_id == 311:  # FY3-弹1-M1
#     v_U = 130.00
#     P_U0 = np.array([6000, -3000, 700])
#     t_release = 0.510
#     t_burst = 4.035
#     angle_deg_U = 175.68
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 312:  # FY3-弹1-M2
#     v_U = 130.00
#     P_U0 = np.array([6000, -3000, 700])
#     t_release = 0.510
#     t_burst = 4.035
#     angle_deg_U = 175.68
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 313:  # FY3-弹1-M3
#     v_U = 130.00
#     P_U0 = np.array([6000, -3000, 700])
#     t_release = 0.510
#     t_burst = 4.035
#     angle_deg_U = 175.68
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 321:  # FY3-弹2-M1
#     v_U = 130.00
#     P_U0 = np.array([6000, -3000, 700])
#     t_release = 4.463
#     t_burst = 7.203
#     angle_deg_U = 175.68
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 322:  # FY3-弹2-M2
#     v_U = 130.00
#     P_U0 = np.array([6000, -3000, 700])
#     t_release = 4.463
#     t_burst = 7.203
#     angle_deg_U = 175.68
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 323:  # FY3-弹2-M3
#     v_U = 130.00
#     P_U0 = np.array([6000, -3000, 700])
#     t_release = 4.463
#     t_burst = 7.203
#     angle_deg_U = 175.68
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 331:  # FY3-弹3-M1
#     v_U = 130.00
#     P_U0 = np.array([6000, -3000, 700])
#     t_release = 9.468
#     t_burst = 13.079
#     angle_deg_U = 175.68
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 332:  # FY3-弹3-M2
#     v_U = 130.00
#     P_U0 = np.array([6000, -3000, 700])
#     t_release = 9.468
#     t_burst = 13.079
#     angle_deg_U = 175.68
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 333:  # FY3-弹3-M3
#     v_U = 130.00
#     P_U0 = np.array([6000, -3000, 700])
#     t_release = 9.468
#     t_burst = 13.079
#     angle_deg_U = 175.68
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# # FY4系列（百位=4）
# elif config_id == 411:  # FY4-弹1-M1
#     v_U = 95.99
#     P_U0 = np.array([11000, 2000, 1800])
#     t_release = 2.765
#     t_burst = 5.816
#     angle_deg_U = 185.79
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 412:  # FY4-弹1-M2
#     v_U = 95.99
#     P_U0 = np.array([11000, 2000, 1800])
#     t_release = 2.765
#     t_burst = 5.816
#     angle_deg_U = 185.79
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 413:  # FY4-弹1-M3
#     v_U = 95.99
#     P_U0 = np.array([11000, 2000, 1800])
#     t_release = 2.765
#     t_burst = 5.816
#     angle_deg_U = 185.79
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 421:  # FY4-弹2-M1
#     v_U = 95.99
#     P_U0 = np.array([11000, 2000, 1800])
#     t_release = 7.204
#     t_burst = 11.836
#     angle_deg_U = 185.79
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 422:  # FY4-弹2-M2
#     v_U = 95.99
#     P_U0 = np.array([11000, 2000, 1800])
#     t_release = 7.204
#     t_burst = 11.836
#     angle_deg_U = 185.79
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 423:  # FY4-弹2-M3
#     v_U = 95.99
#     P_U0 = np.array([11000, 2000, 1800])
#     t_release = 7.204
#     t_burst = 11.836
#     angle_deg_U = 185.79
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 431:  # FY4-弹3-M1
#     v_U = 95.99
#     P_U0 = np.array([11000, 2000, 1800])
#     t_release = 8.660
#     t_burst = 10.916
#     angle_deg_U = 185.79
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 432:  # FY4-弹3-M2
#     v_U = 95.99
#     P_U0 = np.array([11000, 2000, 1800])
#     t_release = 8.660
#     t_burst = 10.916
#     angle_deg_U = 185.79
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 433:  # FY4-弹3-M3
#     v_U = 95.99
#     P_U0 = np.array([11000, 2000, 1800])
#     t_release = 8.660
#     t_burst = 10.916
#     angle_deg_U = 185.79
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# # FY5系列（百位=5）
# elif config_id == 511:  # FY5-弹1-M1
#     v_U = 70.18
#     P_U0 = np.array([13000, -2000, 1300])
#     t_release = 5.171
#     t_burst = 6.831
#     angle_deg_U = 240.35
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 512:  # FY5-弹1-M2
#     v_U = 70.18
#     P_U0 = np.array([13000, -2000, 1300])
#     t_release = 5.171
#     t_burst = 6.831
#     angle_deg_U = 240.35
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 513:  # FY5-弹1-M3
#     v_U = 70.18
#     P_U0 = np.array([13000, -2000, 1300])
#     t_release = 5.171
#     t_burst = 6.831
#     angle_deg_U = 240.35
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 521:  # FY5-弹2-M1
#     v_U = 70.18
#     P_U0 = np.array([13000, -2000, 1300])
#     t_release = 6.599
#     t_burst = 11.931
#     angle_deg_U = 240.35
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 522:  # FY5-弹2-M2
#     v_U = 70.18
#     P_U0 = np.array([13000, -2000, 1300])
#     t_release = 6.599
#     t_burst = 11.931
#     angle_deg_U = 240.35
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 523:  # FY5-弹2-M3
#     v_U = 70.18
#     P_U0 = np.array([13000, -2000, 1300])
#     t_release = 6.599
#     t_burst = 11.931
#     angle_deg_U = 240.35
#     P_M0 = np.array([18000.0, -600.0, 1900.0])

# elif config_id == 531:  # FY5-弹3-M1
#     v_U = 70.18
#     P_U0 = np.array([13000, -2000, 1300])
#     t_release = 8.266
#     t_burst = 9.739
#     angle_deg_U = 240.35
#     P_M0 = np.array([20000.0, 0.0, 2000.0])

# elif config_id == 532:  # FY5-弹3-M2
#     v_U = 70.18
#     P_U0 = np.array([13000, -2000, 1300])
#     t_release = 8.266
#     t_burst = 9.739
#     angle_deg_U = 240.35
#     P_M0 = np.array([19000.0, 600.0, 2100.0])

# elif config_id == 533:  # FY5-弹3-M3
#     v_U = 70.18
#     P_U0 = np.array([13000, -2000, 1300])
#     t_release = 8.266
#     t_burst = 9.739
#     angle_deg_U = 240.35
#     P_M0 = np.array([18000.0, -600.0, 1900.0])
































# -----------------------------
# 1) 常量与初值（按题意/上文约定）
# -----------------------------
g = 9.8           # m/s^2
v_M = 300.0         # 导弹速度
v_c = 3.0           # 云团中心下沉速度
R = 10.0            # 云团半径


T = np.array([0.0, 200.0, 0.0])           # 真目标

angle_rad_U = np.deg2rad(angle_deg_U)                           # 新增: 转换为弧度
u_U = np.array([np.cos(angle_rad_U), np.sin(angle_rad_U), 0.0])  # 新增: 计算单位方向向量

# 导弹指向“假目标原点 (0,0,0)”的单位方向
u_M = -P_M0 / np.linalg.norm(P_M0)


t_fly_to_burst = t_burst - t_release  # 修改: 计算干扰弹从投放到起爆的飞行时间


# 投放点 S（无人机水平等高直线飞行）
# 修改: 根据新的飞行方向、速度和时间计算投放点 S
S = P_U0 + u_U * v_U * t_release

# 起爆点 H0（忽略空气阻力，水平匀速、竖直自由落体）
# 修改: 根据新的参数计算起爆点 H0
horizontal_disp = u_U * v_U * t_fly_to_burst      # 水平位移向量
vertical_disp = -0.5 * g * (t_fly_to_burst ** 2)  # 垂直位移
H0 = S + np.array([horizontal_disp[0], horizontal_disp[1], vertical_disp])


# 只在“起爆后 20 s 的时间窗”内判定相交
t_min = t_burst
t_max = t_burst + 20.0

# 若需限制到“导弹命中假目标”的时刻，也可启用下面两行：
# t_hit = np.linalg.norm(P_M0) / v_M
# t_max = min(t_max, t_hit)

# -----------------------------
# 2) 轨迹/几何辅助函数
# -----------------------------
def P_M(t: float) -> np.ndarray:
    """时刻 t 导弹位置"""
    return P_M0 + v_M * u_M * t

def C(t: float) -> np.ndarray:
    """时刻 t 云团中心位置（起爆后竖直下沉）"""
    # 仅在 t >= t_burst 有意义
    return H0 + np.array([0.0, 0.0, -v_c * (t - t_burst)])

def distance_point_to_segment(P: np.ndarray, Q: np.ndarray, Cc: np.ndarray):
    """
    点 Cc 到线段 PQ 的最小距离；返回 (距离, 线段参数 s_clamped)
    其中 s_clamped ∈ [0,1]，表示最近点为 P + s* (Q-P)
    """
    d = Q - P
    L2 = float(np.dot(d, d))
    if L2 == 0.0:
        return float(np.linalg.norm(Cc - P)), 0.0
    s = float(np.dot(Cc - P, d) / L2)
    s_clamped = min(1.0, max(0.0, s))
    closest = P + s_clamped * d
    return float(np.linalg.norm(Cc - closest)), s_clamped

def dist_to_LOS(t: float) -> float:
    """时刻 t，云心到导弹->目标视线线段的距离"""
    return distance_point_to_segment(P_M(t), T, C(t))[0]

# f(t) = d(t) - R 的符号改变表示进入/离开相交
def f(t: float) -> float:
    return dist_to_LOS(t) - R

# -----------------------------
# 3) 二分法根查找（稳健）
# -----------------------------
def bisect_root(a: float, b: float, max_iter: int = 60, eps: float = 1e-9) -> float:
    """
    在 [a,b] 上对 f(t)=0 做二分；要求 f(a)*f(b) <= 0
    返回根的近似值
    """
    fa, fb = f(a), f(b)
    if fa == 0.0:
        return a
    if fb == 0.0:
        return b
    # 若同号，尽量不调用
    if fa * fb > 0:
        raise ValueError("bisect_root: f(a) 与 f(b) 同号，无法二分。")

    left, right = a, b
    for _ in range(max_iter):
        mid = 0.5 * (left + right)
        fm = f(mid)
        # 收敛或区间极小
        if abs(fm) < eps or (right - left) < 1e-9:
            return mid
        # 选择有异号的一侧
        if fa * fm <= 0:
            right, fb = mid, fm
        else:
            left, fa = mid, fm
    return 0.5 * (left + right)

# -----------------------------
# 4) 事件探测：先粗扫，再精化
# -----------------------------
def find_intervals(t0: float, t1: float, coarse_step: float = 1e-3) -> list:
    """
    在 [t0, t1] 内先用等步长粗扫（默认 1 ms），
    找到 f 从 >0 到 <=0（进入）及反向（离开）的片段，
    再用二分法精化边界。
    返回 [(t_enter, t_exit), ...]
    """
    # 注意：粗步长可按需要调节（如 0.01~0.05 s 亦可）
    ts = np.arange(t0, t1 + coarse_step, coarse_step, dtype=np.float64)
    vals = np.array([f(t) for t in ts], dtype=np.float64)

    intervals = []
    inside = False
    t_enter = None

    for i in range(1, len(ts)):
        f_prev, f_curr = vals[i - 1], vals[i]
        a, b = float(ts[i - 1]), float(ts[i])

        if (not inside) and (f_prev > 0.0) and (f_curr <= 0.0):
            # 进入相交：在 [a,b] 上二分
            t_enter = bisect_root(a, b)
            inside = True

        elif inside and (f_prev <= 0.0) and (f_curr > 0.0):
            # 离开相交：在 [a,b] 上二分
            t_exit = bisect_root(a, b)
            intervals.append((t_enter, t_exit))
            inside = False
            t_enter = None

    # 若到末端仍在相交内，截断于 t1
    if inside and t_enter is not None:
        intervals.append((t_enter, t1))

    return intervals

# -----------------------------
# 5) 主流程：求解并打印结果
# -----------------------------
if __name__ == "__main__":
    intervals = find_intervals(t_min, t_max, coarse_step=0.01)  # 粗扫步长可改
    total = sum(e - s for s, e in intervals)

    print("=== 参数概览 ===")
    print(f"t_burst = {t_burst:.3f} s, 时间窗 = [{t_min:.3f}, {t_max:.3f}] s")
    # 修改: 增加投放点 S 和起爆点 H0 的打印，使其更明确
    print(f"投放位置 S = [{S[0]:.3f}, {S[1]:.3f}, {S[2]:.3f}]")
    print(f"起爆位置 H0 = [{H0[0]:.3f}, {H0[1]:.3f}, {H0[2]:.3f}]")

    print("\n=== 相交通段（视线被遮蔽）=== ")
    if not intervals:
        print("无相交通段。")
    else:
        for k, (s, e) in enumerate(intervals, 1):
            print(f"段 {k}: 进入 = {s:.6f} s, 离开 = {e:.6f} s, 历时 = {e - s:.6f} s")
    print(f"\n遮蔽总时长 = {total:.6f} s")

    # -------------------------
    # 可选：绘制 d(t) 曲线辅助自检
    # -------------------------
    PLOT = True   # 如需图示改为 True
    if PLOT:
        import matplotlib.pyplot as plt
        ts = np.linspace(t_min, t_max, 5000)
        ds = np.array([dist_to_LOS(t) for t in ts])

        plt.figure()
        plt.plot(ts, ds, label="distance to LOS")
        plt.axhline(R, linestyle="--", color="r", label="Cloud Radius (R)") # 修改: 使半径线更清晰
        if intervals: # 修改: 仅在有遮挡时绘制区域和图例
            for i, (s, e) in enumerate(intervals):
                plt.axvspan(s, e, alpha=0.2, color="orange", label="Blocked" if i == 0 else "")
        plt.xlabel("time (s)")
        plt.ylabel("distance (m)")
        plt.title("Distance from Cloud Center to Line-of-Sight") # 修改: 英文标题
        plt.legend()
        plt.grid(True, linestyle=':') # 修改: 增加网格
        plt.tight_layout()
        plt.show()