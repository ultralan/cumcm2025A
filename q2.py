import numpy as np
from scipy.optimize import differential_evolution
import warnings
import math

# --- 1. 常量和初始条件定义 (与之前代码相同) ---
G = 9.8
R_CLOUD = 10.0
V_SINK = 3.0
CLOUD_EFFECTIVE_DURATION = 20.0

P_M1_0 = np.array([20000.0, 0.0, 2000.0])
V_M1 = 300.0
P_FAKE_TARGET = np.array([0.0, 0.0, 0.0])

P_FY1_0 = np.array([17800.0, 0.0, 1800.0])
P_TRUE_TARGET_CENTER = np.array([0.0, 200.0, 5.0])

U_M1 = (P_FAKE_TARGET - P_M1_0) / np.linalg.norm(P_FAKE_TARGET - P_M1_0)
T_IMPACT = np.linalg.norm(P_FAKE_TARGET - P_M1_0) / V_M1

# --- 2. 辅助函数 (与之前代码相同) ---
def get_missile_pos(t):
    return P_M1_0 + U_M1 * V_M1 * t

def dist_point_to_line(p, a, b):
    if np.all(a == b):
        return np.linalg.norm(p - a)
    ap = p - a
    ab = b - a
    distance = np.linalg.norm(np.cross(ap, ab)) / np.linalg.norm(ab)
    return distance

# --- 3. 目标函数 (与之前代码相同) ---
def objective_function(params):
    v_fy1, theta, t_drop, t_fuse = params
    t_det = t_drop + t_fuse

    if t_det >= T_IMPACT:
        return 1000.0

    v_fy1_vec = np.array([v_fy1 * np.cos(theta), v_fy1 * np.sin(theta), 0])
    p_drop = P_FY1_0 + v_fy1_vec * t_drop
    
    dx = v_fy1_vec[0] * t_fuse
    dy = v_fy1_vec[1] * t_fuse
    dz = -0.5 * G * t_fuse**2
    p_det = p_drop + np.array([dx, dy, dz])

    if p_det[2] < 70.0:
        return 1000.0

    effective_time = 0.0
    time_step = 1
    sim_time_points = np.arange(t_det, t_det + CLOUD_EFFECTIVE_DURATION, time_step)
    
    for t in sim_time_points:
        p_missile_t = get_missile_pos(t)
        p_cloud_t = p_det - np.array([0, 0, V_SINK * (t - t_det)])
        distance = dist_point_to_line(p_cloud_t, p_missile_t, P_TRUE_TARGET_CENTER)
        
        if distance <= R_CLOUD:
            effective_time += time_step
            
    return -effective_time

# --- 4. 优化执行 (已更新) ---
if __name__ == "__main__":
    warnings.filterwarnings("ignore", category=RuntimeWarning)
    
    # 通用优化参数
    optimizer_params = {
        'strategy': 'best1bin',
        'maxiter': 1000,
        'popsize': 20,
        'tol': 0.01,
        'recombination': 0.7,
        'mutation': (0.5, 1),
        'disp': False, # 在循环中运行时先关闭实时显示
        'workers': -1
    }

    # --- 第一次优化: 航向角在 180 度附近 ---
    # 比如在 [165, 195] 度范围内搜索
    print("--- 开始第一次优化: 航向角在 180° 附近 ---")
    bounds_180_deg = [
        (70, 140),
        (math.radians(165), math.radians(195)), # 关键约束更新
        (0.1, T_IMPACT - 1),
        (0.1, T_IMPACT - 1)
    ]
    result_180 = differential_evolution(objective_function, bounds_180_deg, **optimizer_params)
    print("第一次优化完成。")

    # --- 第二次优化: 航向角在 0 度附近 ---
    # 比如在 [-15, 15] 度范围内搜索
    print("\n--- 开始第二次优化: 航向角在 0° 附近 ---")
    bounds_0_deg = [
        (70, 140),
        (math.radians(-15), math.radians(15)), # 关键约束更新
        (0.1, T_IMPACT - 1),
        (0.1, T_IMPACT - 1)
    ]
    result_0 = differential_evolution(objective_function, bounds_0_deg, **optimizer_params)
    print("第二次优化完成。")

    # --- 5. 结果比较与展示 ---
    print("\n--- 优化完成，正在比较两次结果 ---")

    # 比较两次优化的结果（目标函数值越小越好）
    if result_180.success and (not result_0.success or result_180.fun < result_0.fun):
        print("\n最优解出现在 180° 附近。")
        result = result_180
    elif result_0.success:
        print("\n最优解出现在 0° 附近。")
        result = result_0
    else:
        print("两次优化均未成功，请检查参数或约束。")
        # 尝试输出一个可能的结果
        result = result_180 if result_180.fun < result_0.fun else result_0

    if result.success or True: # 即使未完全收敛也展示当前最优结果
        best_params = result.x
        max_effective_time = -result.fun
        
        v_fy1_opt, theta_opt, t_drop_opt, t_fuse_opt = best_params
        
        t_det_opt = t_drop_opt + t_fuse_opt
        
        v_fy1_vec_opt = np.array([v_fy1_opt * np.cos(theta_opt), v_fy1_opt * np.sin(theta_opt), 0])
        p_drop_opt = P_FY1_0 + v_fy1_vec_opt * t_drop_opt
        
        dx_opt = v_fy1_vec_opt[0] * t_fuse_opt
        dy_opt = v_fy1_vec_opt[1] * t_fuse_opt
        dz_opt = -0.5 * G * t_fuse_opt**2
        p_det_opt = p_drop_opt + np.array([dx_opt, dy_opt, dz_opt])
        
        print("\n*** 最佳干扰策略 ***")
        print(f"无人机飞行速度: {v_fy1_opt:.3f} m/s")
        print(f"无人机飞行方向 (航向角): {math.degrees(theta_opt):.3f} 度")
        print(f"烟幕干扰弹投放点: ({p_drop_opt[0]:.2f}, {p_drop_opt[1]:.2f}, {p_drop_opt[2]:.2f})")
        print(f"烟幕干扰弹起爆点: ({p_det_opt[0]:.2f}, {p_det_opt[1]:.2f}, {p_det_opt[2]:.2f})")
        print(f"  - 投放时刻: {t_drop_opt:.3f} s")
        print(f"  - 引信时长: {t_fuse_opt:.3f} s")
        print(f"  - 起爆时刻: {t_det_opt:.3f} s")
        print(f"对 M1 的最大有效遮蔽时长: {max_effective_time:.3f} s")