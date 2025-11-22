import numpy as np
from scipy.optimize import differential_evolution, NonlinearConstraint
import time
from multiprocessing import freeze_support

# --- 1. 定义常量和初始条件 ---
G = 9.8
R_CLOUD = 10.0
V_SINK = 3.0
T_EFF = 20.0
P_M1_0 = np.array([20000.0, 0.0, 2000.0])
V_M = 300.0
P_AIM = np.array([0.0, 0.0, 0.0])
P_TARGET = np.array([0.0, 200.0, 5.0])
P_UAVS_0 = {
    1: np.array([17800.0, 0.0, 1800.0]),
    2: np.array([12000.0, 1400.0, 1400.0]),
    3: np.array([6000.0, -3000.0, 700.0]),
}

# --- 2. 预计算导弹轨迹 ---
vec_m1_dir = P_AIM - P_M1_0
dist_m1 = np.linalg.norm(vec_m1_dir)
unit_vec_m1 = vec_m1_dir / dist_m1
VEC_V_M1 = V_M * unit_vec_m1
T_IMPACT = dist_m1 / V_M

# --- 3. 定义核心计算函数 (目标函数) ---
def calculate_obscuration(x):
    total_obscuration_time = 0
    dt = 1 # 模拟时间步长
    time_steps = np.arange(0, T_IMPACT, dt)
    params = x.reshape(3, 4)
    clouds_info = []

    for i in range(3):
        v_ui, theta_i, t_drop_i, t_det_i = params[i]
        uav_id = i + 1
        vec_v_ui = np.array([v_ui * np.cos(theta_i), v_ui * np.sin(theta_i), 0])
        p_drop_i = P_UAVS_0[uav_id] + vec_v_ui * t_drop_i
        dt_fall = t_det_i - t_drop_i

        # 确保dt_fall不为负，虽然约束已经避免了，但这里做个额外保护
        if dt_fall < 0:
            # 这是一个不合法的时间序列，返回一个非常差的目标函数值
            return 1e9 
        
        # 计算起爆点
        p_det_i = p_drop_i + vec_v_ui * dt_fall - np.array([0, 0, 0.5 * G * dt_fall**2])
        
        clouds_info.append({
            'det_time': t_det_i,
            'det_pos': p_det_i,
            'active_end_time': t_det_i + T_EFF
        })

    for t in time_steps:
        is_obscured_at_t = False
        p_m1_t = P_M1_0 + VEC_V_M1 * t
        los_vec = P_TARGET - p_m1_t
        los_len_sq = np.dot(los_vec, los_vec)

        if los_len_sq < 1e-6: # 导弹非常接近目标，LO S向量长度接近0，避免除以零
            continue

        for i in range(3):
            info = clouds_info[i]
            
            # 检查烟幕弹的活动时间范围
            if not (info['det_time'] <= t < info['active_end_time']):
                continue

            # 计算当前时刻云团中心位置 (下沉)
            dt_sink = t - info['det_time']
            # 确保dt_sink不为负，因为只有在起爆后才下沉
            current_dt_sink = max(0, dt_sink) 
            p_cloud_t = info['det_pos'] - np.array([0, 0, V_SINK * current_dt_sink])
            
            # 计算云团中心到视线(LOS)的距离
            cloud_to_missile_vec = p_cloud_t - p_m1_t
            
            cross_prod = np.cross(los_vec, cloud_to_missile_vec)
            dist_to_los = np.sqrt(np.dot(cross_prod, cross_prod)) / np.sqrt(los_len_sq)

            if dist_to_los <= R_CLOUD:
                is_obscured_at_t = True
                break

        if is_obscured_at_t:
            total_obscuration_time += dt
            
    return -total_obscuration_time

# --- 4. 主执行代码 ---
def main():
    print(f"导弹M1预计在 t = {T_IMPACT:.2f} s 后击中假目标。")
    print("-" * 30)

    # 边界条件，与之前保持一致
    bounds = [
        # UAV 1
        (70, 140),          # v_u1
        (170/180*np.pi, np.pi),     # theta_1
        (0.1, T_IMPACT - T_EFF - 1), # t_drop_1 (留出足够反应和下落时间)
        (0.2, T_IMPACT - T_EFF),   # t_det_1
        # UAV 2
        (70, 140),          # v_u2
        (220/180*np.pi, 240/180*np.pi),     # theta_2
        (0.1, T_IMPACT - T_EFF - 1), # t_drop_2
        (0.2, T_IMPACT - T_EFF),   # t_det_2
        # UAV 3
        (70, 140),          # v_u3
        (105/180*np.pi, 120/180*np.pi),     # theta_3
        (0.1, T_IMPACT - T_EFF - 1), # t_drop_3
        (0.2, T_IMPACT - T_EFF),   # t_det_3
    ]

    # 非线性约束: t_det > t_drop
    constraint_funcs = [
        lambda x: x[3] - x[2],  # t_det_1 - t_drop_1
        lambda x: x[7] - x[6],  # t_det_2 - t_drop_2
        lambda x: x[11] - x[10] # t_det_3 - t_drop_3
    ]

    nonlinear_constraints = [
        NonlinearConstraint(fun, 0.1, np.inf) for fun in constraint_funcs
    ]

    print("开始进行差分进化优化... (预计需要较长时间，请耐心等待)")
    start_time = time.time()
    
    result = differential_evolution(
        func=calculate_obscuration,
        bounds=bounds,
        constraints=nonlinear_constraints,
        strategy='best1bin',
        maxiter=1000,      # <--- 进一步增加迭代次数
        popsize=30,        # <--- 进一步增加种群大小
        tol=0.01,
        mutation=(0.5, 1),
        recombination=0.7,
        updating='deferred',
        workers=-1,
        disp=True,
        polish=False # <--- 禁用结果磨光，避免不适用于非平滑函数的警告和潜在扰动
    )
    
    end_time = time.time()
    print(f"\n优化完成，耗时: {end_time - start_time:.2f} s")
    print("-" * 30)

    # <--- 增强结果输出的健壮性检查和调试信息 --->
    if result.x is not None:
        print(f"优化状态: {result.message}")
        print(f"原始优化器返回的最佳参数数组 (result.x):\n{result.x}") # 打印原始数组
        
        if len(result.x) != 12:
            print(f"!!! 警告: 优化器返回的参数数量不正确 ({len(result.x)}), 预期为 12。无法解析决策。")
            return

        max_time = -result.fun
        optimal_params = result.x.reshape(3, 4)
        
        print(f"\n找到的最佳策略对应的最大有效遮蔽总时长: {max_time:.2f} s")
        print("\n详细投放策略如下:")
        
        for i in range(3):
            # 再次检查每个无人机的参数组长度
            if len(optimal_params[i]) != 4:
                print(f"\n--- 警告: 无人机 FY{i+1} 的参数组长度不正确，跳过打印 ---")
                print(f"  原始参数: {optimal_params[i]}")
                continue

            v_ui, theta_i, t_drop_i, t_det_i = optimal_params[i]
            uav_id = i + 1
            print(f"\n--- 无人机 FY{uav_id} ---")
            print(f"  飞行速度: {v_ui:.2f} m/s")
            print(f"  飞行方向: {np.rad2deg(theta_i):.2f} 度 (与x轴正向夹角)")
            print(f"  投放时间 (t_drop): {t_drop_i:.2f} s")
            print(f"  起爆时间 (t_det): {t_det_i:.2f} s")
            print(f"  => 产生有效遮蔽窗口(理论): [{t_det_i:.2f} s, {t_det_i + T_EFF:.2f} s]")
    else:
        print("优化器未能找到任何有效解。")

if __name__ == '__main__':
    freeze_support()
    main()