import numpy as np
import matplotlib.pyplot as plt
import sys
from mpl_toolkits.mplot3d import Axes3D

# 添加 gnss-ins-sim 库的路径（请根据实际情况修改）
sys.path.append('./gnss-ins-sim')

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# 定义 IMU 模型
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

# 创建正方形运动文件
def create_square_motion_file(filename, anomaly=False):
    with open(filename, 'w') as f:
        # 初始位置，静止2秒
        f.write("32, 120, 0, 0, 0, 0, 0, 0, 0\n")
        f.write("1, 0, 0, 0, 0, 0, 0, 2, 1\n")  # 静止2秒
        heading = 0  # 初始航向角
        for i in range(4):
            # 如果不是第一次循环，需要转弯
            if i > 0:
                delta_heading = 90
                f.write(f"3, {delta_heading}, 0, 0, 0, 0, 0, 1, 1\n")
                heading += delta_heading
                heading %= 360  # 确保航向角在0-360度之间

            # 加速到5 m/s，用时2秒
            speed = 5  # 目标速度
            vx = speed * np.cos(np.radians(heading))
            vy = speed * np.sin(np.radians(heading))
            f.write(f"2, 0, 0, 0, {vx}, {vy}, 0, 2, 1\n")

            # 均速前进5秒
            if anomaly and i == 1:
                # 在第二次直线运动时引入异常
                f.write("1, 0, 0, 0, 1, 1, 0, 5, 1\n")  # 增加一个小的横向动作来体现异常
            else:
                f.write("1, 0, 0, 0, 0, 0, 0, 5, 1\n")

            # 减速到0 m/s，用时2秒
            f.write("2, 0, 0, 0, 0, 0, 0, 2, 1\n")

        # 在最后一次转弯后，继续前进，回到起点
        delta_heading = 90
        f.write(f"3, {delta_heading}, 0, 0, 0, 0, 0, 1, 1\n")
        heading += delta_heading
        heading %= 360  # 更新航向角

        # 加速到5 m/s，用2秒
        speed = 5
        vx = speed * np.cos(np.radians(heading))
        vy = speed * np.sin(np.radians(heading))
        f.write(f"2, 0, 0, 0, {vx}, {vy}, 0, 2, 1\n")

        # 均速前进5秒，回到起点
        f.write("1, 0, 0, 0, 0, 0, 0, 5, 1\n")

        # 减速到0 m/s，用2秒
        f.write("2, 0, 0, 0, 0, 0, 0, 2, 1\n")

# 生成无异常和有异常的运动文件
create_square_motion_file('./motion_def_no_anomaly.csv', anomaly=False)
create_square_motion_file('./motion_def_with_anomaly.csv', anomaly=True)

# 设置仿真参数
fs = 100.0    # IMU 采样频率 (Hz)
fs_gps = 1.0  # GPS 采样频率 (Hz)
fs_mag = 10.0 # 磁力计采样频率 (Hz)

# 创建并运行无异常的仿真
sim_no_anomaly = ins_sim.Sim(
    [fs, fs_gps, fs_mag],
    './motion_def_no_anomaly.csv',
    ref_frame=0,
    imu=imu,
    mode=None,
    env=None,
    algorithm=None
)
sim_no_anomaly.run(1)

# 创建并运行有异常的仿真
sim_with_anomaly = ins_sim.Sim(
    [fs, fs_gps, fs_mag],
    './motion_def_with_anomaly.csv',
    ref_frame=0,
    imu=imu,
    mode=None,
    env=None,
    algorithm=None
)
sim_with_anomaly.run(1)

# 提取无异常数据并绘图
ref_pos_no_anomaly = np.array(sim_no_anomaly.get_data(['ref_pos'])[0])
accel_no_anomaly = np.concatenate([sim_no_anomaly.get_data(['accel'])[0][key] for key in sim_no_anomaly.get_data(['accel'])[0]])
gyro_no_anomaly = np.concatenate([sim_no_anomaly.get_data(['gyro'])[0][key] for key in sim_no_anomaly.get_data(['gyro'])[0]])

# 提取有异常数据并绘图
ref_pos_with_anomaly = np.array(sim_with_anomaly.get_data(['ref_pos'])[0])
accel_with_anomaly = np.concatenate([sim_with_anomaly.get_data(['accel'])[0][key] for key in sim_with_anomaly.get_data(['accel'])[0]])
gyro_with_anomaly = np.concatenate([sim_with_anomaly.get_data(['gyro'])[0][key] for key in sim_with_anomaly.get_data(['gyro'])[0]])

# 绘制无异常的轨迹（在XY平面上的3D图）
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(ref_pos_no_anomaly[:, 0], ref_pos_no_anomaly[:, 1], np.zeros_like(ref_pos_no_anomaly[:, 0]), label='No Anomaly', marker='o')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('3D Trajectory - No Anomaly (XY Plane)')
ax.legend()
plt.show()

# 绘制有异常的轨迹（在XY平面上的3D图）
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(ref_pos_with_anomaly[:, 0], ref_pos_with_anomaly[:, 1], np.zeros_like(ref_pos_with_anomaly[:, 0]), label='With Anomaly', marker='o')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('3D Trajectory - With Anomaly (XY Plane)')
ax.legend()
plt.show()

# 绘制无异常的加速度和陀螺仪数据
time_no_anomaly = np.linspace(0, accel_no_anomaly.shape[0] / fs, accel_no_anomaly.shape[0])
plt.figure(figsize=(10, 6))
plt.plot(time_no_anomaly, accel_no_anomaly[:, 0], label='Accel X')
plt.plot(time_no_anomaly, accel_no_anomaly[:, 1], label='Accel Y')
plt.plot(time_no_anomaly, accel_no_anomaly[:, 2], label='Accel Z')
plt.axvspan(0, 2, color='blue', alpha=0.2, label='Static Phase')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.title('Accelerometer Data - No Anomaly')
plt.grid()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time_no_anomaly, gyro_no_anomaly[:, 0], label='Gyro X')
plt.plot(time_no_anomaly, gyro_no_anomaly[:, 1], label='Gyro Y')
plt.plot(time_no_anomaly, gyro_no_anomaly[:, 2], label='Gyro Z')
plt.axvspan(0, 2, color='blue', alpha=0.2, label='Static Phase')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Angular Rate (rad/s)')
plt.title('Gyroscope Data - No Anomaly')
plt.grid()
plt.show()

# 绘制有异常的加速度和陀螺仪数据，标出异常
time_with_anomaly = np.linspace(0, accel_with_anomaly.shape[0] / fs, accel_with_anomaly.shape[0])
plt.figure(figsize=(10, 6))
plt.plot(time_with_anomaly, accel_with_anomaly[:, 0], label='Accel X')
plt.plot(time_with_anomaly, accel_with_anomaly[:, 1], label='Accel Y')
plt.plot(time_with_anomaly, accel_with_anomaly[:, 2], label='Accel Z')
plt.axvspan(0, 2, color='blue', alpha=0.2, label='Static Phase')
plt.axvspan(15, 16, color='red', alpha=0.3, label='Accel Anomaly')  # 加速度计异常
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.title('Accelerometer Data - With Anomaly')
plt.grid()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time_with_anomaly, gyro_with_anomaly[:, 0], label='Gyro X')
plt.plot(time_with_anomaly, gyro_with_anomaly[:, 1], label='Gyro Y')
plt.plot(time_with_anomaly, gyro_with_anomaly[:, 2], label='Gyro Z')
plt.axvspan(0, 2, color='blue', alpha=0.2, label='Static Phase')
plt.axvspan(25, 26, color='orange', alpha=0.3, label='Gyro Anomaly')  # 陀螺仪异常
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Angular Rate (rad/s)')
plt.title('Gyroscope Data - With Anomaly')
plt.grid()
plt.show()
