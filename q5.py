import numpy as np
import pandas as pd
from scipy.optimize import differential_evolution
import os
import time

# ==============================================================================
# 1. 环境设置与常量定义 (无变动)
# ==============================================================================
G = 9.8
V_SINK = np.array([0, 0, -3.0])
R_SMOKE = 10.0
T_EFFECTIVE = 20.0
V_MISSILE = 300.0
T_INTERVAL = 1.0
P_FAKE_TARGET = np.array([0, 0, 0])
P_TRUE_TARGET_BASE = np.array([0, 200.0, 0])
H_TRUE_TARGET = 10.0
TRUE_TARGET_AXIS_BOTTOM = P_TRUE_TARGET_BASE
TRUE_TARGET_AXIS_TOP = P_TRUE_TARGET_BASE + np.array([0, 0, H_TRUE_TARGET])
INITIAL_POS = {
    'M1': np.array([20000.0, 0.0, 2000.0]), 'M2': np.array([19000.0, 600.0, 2100.0]),
    'M3': np.array([18000.0, -600.0, 1900.0]), 'FY1': np.array([17800.0, 0.0, 1800.0]),
    'FY2': np.array([12000.0, 1400.0, 1400.0]), 'FY3': np.array([6000.0, -3000.0, 700.0]),
    'FY4': np.array([11000.0, 2000.0, 1800.0]), 'FY5': np.array([13000.0, -2000.0, 1300.0]),
}
NUM_DRONES = 5
NUM_MISSILES = 3
GRENADES_PER_DRONE = 3
MISSILE_DIRECTIONS = {f'M{j+1}': (P_FAKE_TARGET - INITIAL_POS[f'M{j+1}']) / np.linalg.norm(P_FAKE_TARGET - INITIAL_POS[f'M{j+1}']) for j in range(NUM_MISSILES)}
MISSILE_TOTAL_TIME = {f'M{j+1}': np.linalg.norm(P_FAKE_TARGET - INITIAL_POS[f'M{j+1}']) / V_MISSILE for j in range(NUM_MISSILES)}

CONSTRAINED_VALUES = {
    0: { 'speed': 91.62, 'angle': 3.133, 'deploy_times': [1.21, 2.21, 3.21], 'fuse_times': [None, None, None]},
    1: { 'speed': 101.72, 'angle': -2.082, 'deploy_times': [6.48, 7.94, 21.43], 'fuse_times': [11.36 - 6.48, 15.24 - 7.94, 38.32 - 21.43]},
    2: { 'speed': 136.62, 'angle': 2.329, 'deploy_times': [21.80, 22.80, 23.80], 'fuse_times': [None, None, None]},
    3: { 'speed': 139.003, 'angle': np.deg2rad(271.339), 'deploy_times': [2.171, None, None], 'fuse_times': [14.007 - 2.171, None, None]},
    4: { 'speed': 139.580, 'angle': np.deg2rad(109.407), 'deploy_times': [11.818, None, None], 'fuse_times': [15.440 - 11.818, None, None]}
}
FLUCTUATION = 0.01

# ==============================================================================
# 2. 物理与几何模型 (无变动)
# ==============================================================================
def get_missile_pos(missile_idx, t):
    key = f'M{missile_idx+1}'
    if t > MISSILE_TOTAL_TIME[key]: return P_FAKE_TARGET
    return INITIAL_POS[key] + MISSILE_DIRECTIONS[key] * V_MISSILE * t

def is_obscured(missile_pos, cloud_center):
    p, a, b = cloud_center, TRUE_TARGET_AXIS_BOTTOM, missile_pos
    ap, ab = p - a, b - a
    proj_len, ab_len_sq = np.dot(ap, ab), np.dot(ab, ab)
    if ab_len_sq == 0: return False
    t = proj_len / ab_len_sq
    if t < 0: closest_point = a
    elif t > 1: closest_point = b
    else: closest_point = a + t * ab
    return np.linalg.norm(p - closest_point) <= R_SMOKE

# ==============================================================================
# 3. 目标函数 (无变动)
# ==============================================================================
def objective_function(x):
    num_drone_vars = NUM_DRONES * 2
    drone_params, grenade_params = x[:num_drone_vars], x[num_drone_vars:]
            
    drone_strategies = []
    for i in range(NUM_DRONES):
        speed, angle = drone_params[i*2], drone_params[i*2 + 1]
        drone_strategies.append({'speed': speed, 'angle': angle, 'dir_vec': np.array([np.cos(angle), np.sin(angle), 0]), 'init_pos': INITIAL_POS[f'FY{i+1}']})
    
    grenade_strategies = []
    for i in range(NUM_DRONES):
        drone_grenade_times = [grenade_params[(i * GRENADES_PER_DRONE + k) * 2] for k in range(GRENADES_PER_DRONE)]
        sorted_times = sorted(drone_grenade_times)
        repaired_times = [sorted_times[0]]
        for k in range(1, GRENADES_PER_DRONE):
            repaired_times.append(max(sorted_times[k], repaired_times[k-1] + T_INTERVAL))
        for k in range(GRENADES_PER_DRONE):
            idx = (i * GRENADES_PER_DRONE + k) * 2
            grenade_strategies.append({'drone_idx': i, 't_deploy': repaired_times[k], 't_fuse': grenade_params[idx + 1]})
            
    events = []
    for gs in grenade_strategies:
        drone = drone_strategies[gs['drone_idx']]
        t_dep, t_fuse = gs['t_deploy'], gs['t_fuse']
        p_deploy = drone['init_pos'] + drone['dir_vec'] * drone['speed'] * t_dep
        v_deploy = drone['dir_vec'] * drone['speed']
        t_detonate = t_dep + t_fuse
        p_detonate = p_deploy + v_deploy * t_fuse + 0.5 * np.array([0, 0, -G]) * t_fuse**2
        if p_detonate[2] > 0:
            events.append({'t_detonate': t_detonate, 'p_detonate': p_detonate, 't_end': t_detonate + T_EFFECTIVE})
    
    simulation_end_time = max(MISSILE_TOTAL_TIME.values())
    time_points = np.arange(0, simulation_end_time, 1.0)
    total_obscured_time = 0
    for t in time_points:
        active_clouds = []
        for event in events:
            if event['t_detonate'] <= t < event['t_end']:
                cloud_pos = event['p_detonate'] + V_SINK * (t - event['t_detonate'])
                if cloud_pos[2] > 0: active_clouds.append(cloud_pos)
        if not active_clouds: continue
        blocked_missiles_this_step = set()
        for m_idx in range(NUM_MISSILES):
            if MISSILE_TOTAL_TIME[f'M{m_idx+1}'] < t: continue
            missile_pos = get_missile_pos(m_idx, t)
            for cloud_pos in active_clouds:
                if is_obscured(missile_pos, cloud_pos):
                    blocked_missiles_this_step.add(m_idx)
                    break
        total_obscured_time += len(blocked_missiles_this_step) * 1.0
    return -total_obscured_time

# ==============================================================================
# 4. 主执行模块
# ==============================================================================

def save_strategy_to_excel(strategy_vector, filename):
    print(f"正在生成并保存策略到 {filename} ...")
    output_data = []
    num_drone_vars = NUM_DRONES * 2
    drone_params, grenade_params = strategy_vector[:num_drone_vars], strategy_vector[num_drone_vars:]
    
    drone_strategies_final = []
    for i in range(NUM_DRONES):
        speed, angle = drone_params[i*2], drone_params[i*2 + 1]
        drone_strategies_final.append({'speed': speed, 'angle': angle})

    repaired_grenade_info = []
    for i in range(NUM_DRONES):
        drone_grenade_times_raw = [grenade_params[(i*GRENADES_PER_DRONE + k)*2] for k in range(GRENADES_PER_DRONE)]
        sorted_times = sorted(drone_grenade_times_raw)
        repaired_times = [sorted_times[0]]
        for k in range(1, GRENADES_PER_DRONE):
            repaired_times.append(max(sorted_times[k], repaired_times[k-1] + T_INTERVAL))
        for k in range(GRENADES_PER_DRONE):
            idx = (i*GRENADES_PER_DRONE + k)*2
            repaired_grenade_info.append({'drone_idx': i, 'grenade_k': k+1, 
                                           't_deploy': repaired_times[k], 't_fuse': grenade_params[idx+1]})

    for info in repaired_grenade_info:
        drone_idx = info['drone_idx']
        drone = drone_strategies_final[drone_idx]
        speed, angle = drone['speed'], drone['angle']
        dir_vec = np.array([np.cos(angle), np.sin(angle), 0])
        
        t_dep, t_fuse = info['t_deploy'], info['t_fuse']
        p_deploy = INITIAL_POS[f'FY{drone_idx+1}'] + dir_vec * speed * t_dep
        t_detonate = t_dep + t_fuse
        p_detonate = p_deploy + (dir_vec*speed)*t_fuse + 0.5*np.array([0,0,-G])*t_fuse**2

        obscuration_per_missile = np.zeros(NUM_MISSILES)
        for t in np.arange(t_detonate, t_detonate + T_EFFECTIVE, 0.5):
            cloud_pos = p_detonate + V_SINK * (t - t_detonate)
            if cloud_pos[2] <= 0: continue
            for m_idx in range(NUM_MISSILES):
                if MISSILE_TOTAL_TIME[f'M{m_idx+1}'] < t: continue
                missile_pos = get_missile_pos(m_idx, t)
                if is_obscured(missile_pos, cloud_pos):
                    obscuration_per_missile[m_idx] += 0.5
        
        target_missile_idx = np.argmax(obscuration_per_missile) if np.sum(obscuration_per_missile) > 0 else -1
        target_missile = f'M{target_missile_idx + 1}' if target_missile_idx != -1 else "无"
        
        output_data.append({
            '无人机编号': f'FY{drone_idx+1}', '烟幕弹序号': info['grenade_k'],
            '主要干扰对象': target_missile, '无人机飞行速度(m/s)': f'{speed:.2f}',
            '无人机飞行方向(rad)': f'{angle:.3f}', '投放时刻(s)': f'{t_dep:.2f}',
            '投放点坐标(x,y,z)': f'({p_deploy[0]:.1f}, {p_deploy[1]:.1f}, {p_deploy[2]:.1f})',
            '引信时长(s)': f'{t_fuse:.2f}', '起爆时刻(s)': f'{t_detonate:.2f}',
            '起爆点坐标(x,y,z)': f'({p_detonate[0]:.1f}, {p_detonate[1]:.1f}, {p_detonate[2]:.1f})'
        })
        
    df = pd.DataFrame(output_data)
    desired_order = ['无人机编号', '烟幕弹序号', '主要干扰对象', '无人机飞行速度(m/s)', '无人机飞行方向(rad)', 
                     '投放时刻(s)', '投放点坐标(x,y,z)', '引信时长(s)', '起爆时刻(s)', '起爆点坐标(x,y,z)']
    df = df[desired_order]
    
    df.to_excel(filename, index=False)
    print(f"已成功保存 {filename}")


if __name__ == "__main__":
    start_time = time.time()
    
    print("正在应用所有带1%波动的精细化约束...")
    bounds = []
    
    # --- 边界1: 5架无人机的速度和角度 ---
    for i in range(NUM_DRONES):
        drone_constraints = CONSTRAINED_VALUES.get(i)
        
        # ###################################################################### #
        # ##               代码修改部分 START (修正边界计算)                    ## #
        # ###################################################################### #
        
        # 速度边界
        if drone_constraints and drone_constraints.get('speed') is not None:
            center = drone_constraints['speed']
            delta = abs(center) * FLUCTUATION
            bounds.append((center - delta, center + delta))
            print(f"为 FY{i+1} 设置速度波动边界: [{bounds[-1][0]:.2f}, {bounds[-1][1]:.2f}]")
        else:
            bounds.append((70, 140))
            print(f"为 FY{i+1} 设置默认速度边界: [70, 140]")
            
        # 角度边界
        if drone_constraints and drone_constraints.get('angle') is not None:
            center = drone_constraints['angle']
            delta = abs(center) * FLUCTUATION
            bounds.append((center - delta, center + delta))
            print(f"为 FY{i+1} 设置角度波动边界: [{bounds[-1][0]:.3f}, {bounds[-1][1]:.3f}]")
        else:
            bounds.append((-np.pi, np.pi))
            print(f"为 FY{i+1} 设置默认角度边界: [-pi, pi]")

    # --- 边界2: 15枚烟幕弹的投放和引信时长 ---
    max_deploy_time = min(MISSILE_TOTAL_TIME.values())
    for i in range(NUM_DRONES):
        for k in range(GRENADES_PER_DRONE):
            drone_constraints = CONSTRAINED_VALUES.get(i)
            
            # 投放时间边界
            if drone_constraints and drone_constraints['deploy_times'][k] is not None:
                center = drone_constraints['deploy_times'][k]
                delta = abs(center) * FLUCTUATION
                bounds.append((center - delta, center + delta))
                print(f"为 FY{i+1}-弹{k+1} 设置投放时间波动边界: [{bounds[-1][0]:.2f}, {bounds[-1][1]:.2f}]")
            else:
                bounds.append((1.5, max_deploy_time))
                print(f"为 FY{i+1}-弹{k+1} 设置默认投放时间边界: [1.5, {max_deploy_time:.2f}]")

            # 引信时长边界
            if drone_constraints and drone_constraints['fuse_times'][k] is not None:
                center = drone_constraints['fuse_times'][k]
                delta = abs(center) * FLUCTUATION
                bounds.append((center - delta, center + delta))
                print(f"为 FY{i+1}-弹{k+1} 设置引信时长波动边界: [{bounds[-1][0]:.2f}, {bounds[-1][1]:.2f}]")
            else:
                bounds.append((1.0, 30.0))
                print(f"为 FY{i+1}-弹{k+1} 设置默认引信时长边界: [1.0, 30.0]")
                
    # ###################################################################### #
    # ##                         代码修改部分 END                           ## #
    # ###################################################################### #

    print(f"\n总计待优化变量数量: {len(bounds)}")

    print("\n" + "="*50)
    print("开始单次深度优化，以寻找最优解...")
    print("="*50)

    result = differential_evolution(
        objective_function, bounds, strategy='best1bin',
        maxiter=30, popsize=40, tol=0.01, recombination=0.7,
        mutation=(0.5, 1), disp=True, updating='deferred', workers=-1
    )

    best_score = -result.fun if result.fun < 1e8 else 0
    best_strategy_vector = result.x
    
    print("\n" + "#"*60)
    print(f"##  优化完成。")
    print(f"##  找到的最优总遮蔽时间: {best_score:.4f} s")
    print("#"*60 + "\n")

    print("最终全局最优策略详情 (已保存为 'result3.xlsx'):")
    save_strategy_to_excel(best_strategy_vector, "result3.xlsx")
    
    end_time = time.time()
    print(f"\n总耗时: {(end_time - start_time) / 60:.2f} 分钟")