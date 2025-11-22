import numpy as np
from scipy.optimize import differential_evolution, NonlinearConstraint
import time

# --- 常量和初始状态 (与之前相同) ---
G = 9.8
P_M1_0 = np.array([20000.0, 0.0, 2000.0])
V_M1_SPEED = 300.0
P_FAKE_TARGET = np.array([0.0, 0.0, 0.0])
P_REAL_CENTER = np.array([0.0, 200.0, 5.0])
P_FY1_0 = np.array([17800.0, 0.0, 1800.0])
R_CLOUD = 10.0
T_CLOUD_EFFECTIVE = 20.0
V_SINK = 3.0
d_m1 = P_FAKE_TARGET - P_M1_0
V_M1_VEC = (d_m1 / np.linalg.norm(d_m1)) * V_M1_SPEED

# --- 辅助函数 (与之前相同) ---
def distance_point_to_segment(point, seg_start, seg_end):
    seg_vec = seg_end - seg_start
    point_vec = point - seg_start
    seg_len_sq = np.dot(seg_vec, seg_vec)
    if seg_len_sq == 0.0:
        return np.linalg.norm(point_vec)
    t = max(0, min(1, np.dot(point_vec, seg_vec) / seg_len_sq))
    projection = seg_start + t * seg_vec
    return np.linalg.norm(point - projection)

# --- 目标函数 (现在不再需要内部检查约束) ---
def objective_function(x):
    v_uav, theta_uav, t_drop1, t_fuse1, t_drop2, t_fuse2, t_drop3, t_fuse3 = x
    
    v_uav_vec = np.array([v_uav * np.cos(theta_uav), v_uav * np.sin(theta_uav), 0])
    
    t_det = np.array([t_drop1 + t_fuse1, t_drop2 + t_fuse2, t_drop3 + t_fuse3])

    p_drops = np.array([P_FY1_0 + v_uav_vec * t_drop1,
                        P_FY1_0 + v_uav_vec * t_drop2,
                        P_FY1_0 + v_uav_vec * t_drop3])

    p_dets = np.array([
        p_drops[0] + v_uav_vec * t_fuse1 + np.array([0, 0, -0.5 * G * t_fuse1**2]),
        p_drops[1] + v_uav_vec * t_fuse2 + np.array([0, 0, -0.5 * G * t_fuse2**2]),
        p_drops[2] + v_uav_vec * t_fuse3 + np.array([0, 0, -0.5 * G * t_fuse3**2])
    ])
    
    total_effective_time = 0.0
    simulation_duration = 70 # 仿真70秒足够
    time_step = 1
    
    for t in np.arange(0, simulation_duration, time_step):
        p_m1_t = P_M1_0 + V_M1_VEC * t
        
        is_screened_this_step = False
        for i in range(3):
            if t_det[i] <= t <= t_det[i] + T_CLOUD_EFFECTIVE:
                p_cloud_t = p_dets[i] - np.array([0, 0, V_SINK * (t - t_det[i])])
                dist = distance_point_to_segment(p_cloud_t, p_m1_t, P_REAL_CENTER)
                if dist <= R_CLOUD:
                    is_screened_this_step = True
                    break
        
        if is_screened_this_step:
            total_effective_time += time_step
            
    return -total_effective_time


# --- 1. 核心修正：定义非线性约束函数 ---
# 约束的标准形式是 c(x) >= 0
def constraint_func(x):
    # x = [v_uav, theta_uav, t_drop1, t_fuse1, t_drop2, t_fuse2, t_drop3, t_fuse3]
    # 索引:             0,         1,       2,       3,       4,       5,       6,       7
    
    # 约束1: t_drop2 >= t_drop1 + 1  =>  t_drop2 - t_drop1 - 1 >= 0
    c1 = x[4] - x[2] - 1.0
    
    # 约束2: t_drop3 >= t_drop2 + 1  =>  t_drop3 - t_drop2 - 1 >= 0
    c2 = x[6] - x[4] - 1.0
    
    # 约束3: t_det1 <= 20 => 20 - (t_drop1 + t_fuse1) >= 0
    c3 = 20.0 - (x[2] + x[3])
    
    # 约束4: t_det2 <= 20 => 20 - (t_drop2 + t_fuse2) >= 0
    c4 = 20.0 - (x[4] + x[5])
    
    # 约束5: t_det3 <= 20 => 20 - (t_drop3 + t_fuse3) >= 0
    c5 = 20.0 - (x[6] + x[7])

    return [c1, c2, c3, c4, c5]

# 根据约束函数创建NonlinearConstraint对象
# 每个约束的下限都是0，上限是无穷大(np.inf)
nonlinear_constraint = NonlinearConstraint(constraint_func, 0, np.inf)


# --- 4. 设置优化参数并运行 ---
if __name__ == "__main__":
    # --- 2. 角度约束更新 ---
    # ±15度 = π/12
    angle_offset = np.pi /180
    theta_min = np.pi - angle_offset
    theta_max = np.pi
    
    t_op_max = 20.0
    
    bounds = [
        (70, 140),              # v_uav (m/s)
        (theta_min, theta_max), # theta_uav (rad)
        (0, t_op_max),          # t_drop1 (s)
        (0, t_op_max),          # t_fuse1 (s)
        (0, t_op_max),          # t_drop2 (s)
        (0, t_op_max),          # t_fuse2 (s)
        (0, t_op_max),          # t_drop3 (s)
        (0, t_op_max)           # t_fuse3 (s)
    ]

    print("开始优化求解 (已修正约束处理方式)...")
    print("本次运行时间会显著增长，预计需要几分钟，请耐心等待...")
    start_time = time.time()
    
    result = differential_evolution(
        objective_function, 
        bounds,
        # 将修正后的约束传入优化器
        constraints=nonlinear_constraint,
        maxiter=1000, # 增加迭代次数以获得更好收敛
        popsize=25,   # 增加种群大小以增强全局搜索
        disp=True,
        workers=-1
    )
    
    end_time = time.time()
    print(f"\n优化完成，耗时: {end_time - start_time:.2f} 秒")

    if result.success:
        optimal_params = result.x
        max_time = -result.fun
        
        print("\n找到最优策略！")
        print(f"最大有效遮蔽总时长: {max_time:.2f} 秒")
        print("-" * 30)
        print("无人机飞行策略:")
        print(f"  - 飞行速度 (v_uav): {optimal_params[0]:.2f} m/s")
        print(f"  - 飞行方向 (theta_uav): {np.rad2deg(optimal_params[1]):.2f} 度")
        print("-" * 30)
        print("干扰弹投放策略:")
        print(f"  - 第1枚: 投放时间={optimal_params[2]:.2f}s, 引信时间={optimal_params[3]:.2f}s, 起爆时间={optimal_params[2]+optimal_params[3]:.2f}s")
        print(f"  - 第2枚: 投放时间={optimal_params[4]:.2f}s, 引信时间={optimal_params[5]:.2f}s, 起爆时间={optimal_params[4]+optimal_params[5]:.2f}s")
        print(f"  - 第3枚: 投放时间={optimal_params[6]:.2f}s, 引信时间={optimal_params[7]:.2f}s, 起爆时间={optimal_params[6]+optimal_params[7]:.2f}s")
    else:
        print("\n优化未成功收敛，可能需要调整参数或增加迭代次数。")
        print(f"错误信息: {result.message}")