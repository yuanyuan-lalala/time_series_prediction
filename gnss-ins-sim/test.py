import numpy as np
import matplotlib.pyplot as plt
import sys

# 添加 gnss-ins-sim 库的路径（请根据实际情况修改）
sys.path.append('./gnss-ins-sim')

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# 第一步：定义 IMU 模型，加入适当的零偏和噪声
imu_err = {
    'gyro_b': np.array([0.01, 0.01, 0.01]),         # 陀螺仪零偏 (度/小时)
    'gyro_arw': np.array([0.1, 0.1, 0.1]),          # 陀螺仪角度随机游走 (度/√小时)
    'gyro_b_stability': np.array([0.1, 0.1, 0.1]),  # 陀螺仪零偏稳定性 (度/小时)
    'gyro_b_corr': np.array([100.0, 100.0, 100.0]), # 零偏相关时间 (s)
    'accel_b': np.array([0.02, 0.02, 0.02]),        # 加速度计零偏 (m/s^2)
    'accel_vrw': np.array([0.02, 0.02, 0.02]),      # 加速度计随机游走 (m/s/√小时)
    'accel_b_stability': np.array([0.001, 0.001, 0.001]), # 加速度计零偏稳定性 (m/s^2)
    'accel_b_corr': np.array([200.0, 200.0, 200.0]), # 零偏相关时间 (s)
}

# 创建 IMU 对象
imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=True)

# 第二步：创建包含重力的运动定义文件
motion_def_file = './motion_def.csv'
with open(motion_def_file, 'w') as f:
    # 初始化位置，静止20秒（包含重力影响）
    f.write("32, 45, 0, 0, 0, 0, 0, 0, -9.81\n")  # 初始位置：包含重力加速度
    f.write("1, 0, 0, 0, 0, 0, 0, 20, 1\n")  # 静止20秒

    # 正常运动：以5 m/s的速度前进60秒
    f.write("5, 0, 0, 0, 5, 0, 0, 60, 1\n")

    # 异常运动1：在正常运动期间发生突然的大角速度变化（模拟陀螺仪异常）
    f.write("5, 0, 0, 0, 5, 0, 100, 5, 1\n")  # wz = 100 rad/s，持续5秒

    # 正常运动：继续以5 m/s的速度前进30秒
    f.write("5, 0, 0, 0, 5, 0, 0, 30, 1\n")

    # 异常运动2：在正常运动期间发生突然的大加速度（模拟加速度计异常）
    f.write("5, 0, 0, 0, 1000, 0, 0, 5, 1\n")  # ax = 1000 m/s^2，持续5秒

    # 正常运动：继续以5 m/s的速度前进19秒
    f.write("5, 0, 0, 0, 5, 0, 0, 19, 1\n")

    # 静止20秒
    f.write("1, 0, 0, 0, 0, 0, 0, 20, 1\n")

# 第三步：创建仿真对象并运行仿真
fs = 100.0    # IMU 采样频率 (Hz)
fs_gps = 1.0  # GPS 采样频率 (Hz)
fs_mag = 10.0 # 磁力计采样频率 (Hz)

sim = ins_sim.Sim(
    [fs, fs_gps, fs_mag],
    motion_def_file,
    ref_frame=0,
    imu=imu,
    mode=None,
    env=None,
    algorithm=None
)

# 运行仿真
sim.run(5)
print("Simulation completed.")

# 第四步：提取数据
# 获取时间序列数据
time_data = sim.get_data(['time'])
time = np.array(time_data[0]).reshape(-1) if time_data else None

# 获取陀螺仪数据并拼接分段数据
gyro_data = sim.get_data(['gyro'])
if gyro_data:
    gyro_segments = [seg for key in gyro_data[0] for seg in [gyro_data[0][key]]]
    gyro = np.concatenate(gyro_segments)
else:
    gyro = None

# 获取加速度计数据并拼接分段数据
accel_data = sim.get_data(['accel'])
if accel_data:
    accel_segments = [seg for key in accel_data[0] for seg in [accel_data[0][key]]]
    accel = np.concatenate(accel_segments)
else:
    accel = None

# 生成与 gyro 和 accel 数据相同长度的时间序列
num_samples = gyro.shape[0]
time_expanded = np.linspace(0, num_samples / fs, num_samples)

# 第五步：绘制结果
# 绘制加速度计数据
plt.figure(figsize=(10, 6))
plt.plot(time_expanded, accel[:, 0], label='Accel X')
plt.plot(time_expanded, accel[:, 1], label='Accel Y')
plt.plot(time_expanded, accel[:, 2], label='Accel Z')
# 标记静止（重力）区间和异常区间
plt.axvspan(0, 20, color='blue', alpha=0.2, label='Static Phase (Gravity Only)')
plt.axvspan(85, 90, color='red', alpha=0.3, label='Gyro Anomaly')
plt.axvspan(125, 130, color='orange', alpha=0.3, label='Accel Anomaly')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.title('Accelerometer Data with Gravity and Anomalies')
plt.grid()
plt.show()

# 绘制陀螺仪数据
plt.figure(figsize=(10, 6))
plt.plot(time_expanded, gyro[:, 0], label='Gyro X')
plt.plot(time_expanded, gyro[:, 1], label='Gyro Y')
plt.plot(time_expanded, gyro[:, 2], label='Gyro Z')
# 标记静止（重力）区间和异常区间
plt.axvspan(0, 20, color='blue', alpha=0.2, label='Static Phase (Gravity Only)')
plt.axvspan(85, 90, color='red', alpha=0.3, label='Gyro Anomaly')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Angular Rate (rad/s)')
plt.title('Gyroscope Data with Anomalies')
plt.grid()
plt.show()
