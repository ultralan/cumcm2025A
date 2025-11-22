import numpy as np

# --- 第一步：建立坐标系与初始化参数 ---

# 坐标（单位：米）
P_false_target = np.array([0.0, 0.0, 0.0])
P_true_target_center = np.array([0.0, 200.0, 5.0]) # 将真目标近似为其几何中心
M1_0 = np.array([20000.0, 0.0, 2000.0]) # 导弹M1初始位置
FY1_0 = np.array([17800.0, 0.0, 1800.0]) # 无人机FY1初始位置

# 速度（单位：米/秒）
v_M1_scalar = 300.0 # 导弹飞行速度
v_FY1_scalar = 120.0 # 无人机飞行速度

# 烟幕参数
R_smoke = 10.0  # 烟幕有效半径
v_smoke_sink_scalar = 3.0 # 烟幕下沉速度
v_smoke_sink_vector = np.array([0.0, 0.0, -v_smoke_sink_scalar])
t_smoke_duration = 20.0 # 烟幕有效持续时间

# 问题1给定的时间（单位：秒）
t_drop = 1.5  # 无人机投弹时间
t_fuse = 3.6  # 引信时间

# 物理常数
g = 9.8  # 重力加速度 (m/s^2)

# --- 第二步：计算关键事件的时刻与位置 ---

# 1. 计算导弹M1的运动轨迹
# 导弹飞行方向为初始位置指向假目标
u_M1 = (P_false_target - M1_0) / np.linalg.norm(P_false_target - M1_0)
v_M1_vector = v_M1_scalar * u_M1

def get_missile_pos(t):
    """根据时间t计算导弹M1的位置"""
    return M1_0 + v_M1_vector * t

# 2. 计算无人机FY1的运动轨迹 (等高度飞行)
# 无人机在水平面(xy平面)上朝向假目标飞行
u_FY1_xy = (P_false_target[:2] - FY1_0[:2]) / np.linalg.norm(P_false_target[:2] - FY1_0[:2])
# 其三维方向向量的z分量为0，以保证等高度飞行
u_FY1_vector = np.array([u_FY1_xy[0], u_FY1_xy[1], 0.0])
v_FY1_vector = v_FY1_scalar * u_FY1_vector

def get_drone_pos(t):
    """根据时间t计算无人机FY1的位置"""
    return FY1_0 + v_FY1_vector * t

# 3. 计算烟幕弹投放点
P_drop = get_drone_pos(t_drop)
# 烟幕弹脱离无人机后，初始速度与无人机相同
v_bomb_initial = v_FY1_vector

# 4. 计算烟幕弹起爆点
# 烟幕弹起爆时刻
t_detonate = t_drop + t_fuse
# 烟幕弹在重力作用下运动
# 使用抛体运动公式计算从投放到起爆的位移
displacement_projectile = v_bomb_initial * t_fuse + 0.5 * np.array([0.0, 0.0, -g]) * t_fuse**2
P_detonate = P_drop + displacement_projectile


# --- 第三步 & 第四步：建立遮蔽模型并求解时间区间 ---

def point_to_line_segment_distance(p, a, b):
    """
    计算点p到线段ab的最小距离
    p: 烟幕中心点
    a: 线段起点 (导弹位置)
    b: 线段终点 (真目标中心)
    """
    ab = b - a
    ap = p - a
    
    # 计算点p在线段ab方向上的投影比例
    dot_product = np.dot(ap, ab)
    
    # 避免除以零（虽然在此问题中ab的长度远不为零）
    squared_len_ab = np.dot(ab, ab)
    if squared_len_ab == 0:
        return np.linalg.norm(ap)
        
    t = dot_product / squared_len_ab
    
    # 如果投影点在线段外部，则最近点为线段端点
    if t < 0.0:
        return np.linalg.norm(p - a) # 最近点是a
    elif t > 1.0:
        return np.linalg.norm(p - b) # 最近点是b
    
    # 如果投影点在线段内部，则计算点到直线的垂直距离
    closest_point_on_line = a + t * ab
    return np.linalg.norm(p - closest_point_on_line)

def solve_obscuration_time():
    """
    通过数值扫描的方式求解有效遮蔽时间段
    """
    t_start_obscuration = -1.0
    t_end_obscuration = -1.0
    
    # 设置扫描的时间步长，步长越小结果越精确
    time_step = 0.01
    
    # 确定求解的时间区间：从烟幕起爆到烟幕失效
    scan_start_time = t_detonate
    scan_end_time = t_detonate + t_smoke_duration
    
    # 遍历求解区间内的每一个时刻点
    for t in np.arange(scan_start_time, scan_end_time, time_step):
        # 获取当前时刻导弹和烟幕中心的位置
        missile_pos = get_missile_pos(t)
        
        time_since_detonation = t - t_detonate
        smoke_center_pos = P_detonate + v_smoke_sink_vector * time_since_detonation
        
        # 计算导弹到真目标的视线是否被遮挡
        # 即判断烟幕中心到视线(线段)的距离是否小于烟幕半径
        distance_to_los = point_to_line_segment_distance(
            smoke_center_pos, missile_pos, P_true_target_center
        )
        
        is_obscured_now = (distance_to_los <= R_smoke)
        
        if is_obscured_now and t_start_obscuration < 0:
            # 记录第一个被遮挡的时刻
            t_start_obscuration = t
            
        if is_obscured_now:
            # 只要被遮挡，就不断更新最后被遮挡的时刻
            t_end_obscuration = t
            
    if t_start_obscuration > 0:
        duration = t_end_obscuration - t_start_obscuration
        return t_start_obscuration, t_end_obscuration, duration
    else:
        # 如果从未被遮挡，则返回0时长
        return None, None, 0

# --- 执行求解并输出结果 ---
if __name__ == "__main__":
    start_time, end_time, duration = solve_obscuration_time()

    print("--- 求解问题1：计算结果 ---")
    print(f"无人机投弹时刻: {t_drop:.3f} s")
    print(f"烟幕弹起爆时刻: {t_detonate:.3f} s")
    print(f"烟幕弹起爆位置 P_detonate: ({P_detonate[0]:.3f}, {P_detonate[1]:.3f}, {P_detonate[2]:.3f}) m")
    print("-" * 30)
    
    if duration > 0:
        print(f"有效遮蔽开始时刻: {start_time:.3f} s")
        print(f"有效遮蔽结束时刻: {end_time:.3f} s")
        print(f"总有效遮蔽时长: {duration:.3f} s")
    else:
        print("在烟幕有效时间内，未能形成有效遮蔽。")