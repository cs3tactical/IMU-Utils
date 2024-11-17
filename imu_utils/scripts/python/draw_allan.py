import numpy as np
import matplotlib.pyplot as plt

dt = np.loadtxt('../../data/data_imu_gyr_t.txt')
data_x = np.loadtxt('../../data/data_imu_gyr_x.txt')
data_y = np.loadtxt('../../data/data_imu_gyr_y.txt')
data_z = np.loadtxt('../../data/data_imu_gyr_z.txt')

data_sim_x = np.loadtxt('../../data/data_imu_sim_gyr_x.txt')
data_sim_y = np.loadtxt('../../data/data_imu_sim_gyr_y.txt')
data_sim_z = np.loadtxt('../../data/data_imu_sim_gyr_z.txt')

plt.figure()
plt.loglog(dt, data_x, label='x')
plt.loglog(dt, data_y, label='y')
plt.loglog(dt, data_z, label='z')
plt.xlabel('Time: sec')
plt.ylabel('Sigma: deg/h')
plt.grid(True)
plt.legend()
plt.show()

plt.figure()
plt.loglog(dt, data_sim_x, label='x')
plt.loglog(dt, data_sim_y, label='y')
plt.loglog(dt, data_sim_z, label='z')
plt.xlabel('Time: sec')
plt.ylabel('Sigma: deg/h')
plt.grid(True)
plt.legend()
plt.show()
