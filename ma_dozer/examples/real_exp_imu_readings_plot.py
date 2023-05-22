import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def closet_init_time(table: pd.DataFrame, init_t):
    line_num = None
    t = table['time'].to_numpy(dtype=int)
    idx = np.argmin(np.abs(t - init_t))
    if idx is not None:
        line_num = idx
    return line_num


def main():
    exp_dir_path = '../data//exp_traj2/'

    cam_gt_path = exp_dir_path + 'camera_gt_meas.csv'
    cam_df = pd.read_csv(cam_gt_path)

    imu_path = exp_dir_path + 'imu_meas.csv'
    # with open(imu_path, 'r') as f:
    #     init_time = int(f.readline().split('=')[-1])

    imu_df = pd.read_csv(imu_path)

    # cut_line = closet_init_time(imu_df, init_time)
    # imu_df = imu_df[cut_line:]
    # imu_df = imu_df.reset_index(drop=True)

    # cut_line = closet_init_time(cam_df, init_time)
    # cam_df = cam_df[cut_line:]
    # cam_df = cam_df.reset_index(drop=True)

    fig0 = plt.figure()

    ax0 = plt.subplot(611)
    ax0.plot((imu_df['time']-imu_df['time'][0])*1e-9, imu_df['x'], label='x')
    ax0.set_xlabel('t [sec]')
    ax0.set_ylabel(r'specific force [m/$s^2$]')
    ax0.grid(True)
    ax0.legend()
    ax1 = plt.subplot(612, sharex=ax0)
    ax1.plot((cam_df['time'] - cam_df['time'][0]) * 1e-9, cam_df['x'], 'r', label='x')
    ax1.set_xlabel('t [sec]')
    ax1.set_ylabel(r'position [cm]')
    ax1.grid(True)
    ax1.legend()

    ax0 = plt.subplot(613)
    ax0.plot((imu_df['time'] - imu_df['time'][0]) * 1e-9, imu_df['y'], label='y')
    ax0.set_xlabel('t [sec]')
    ax0.set_ylabel(r'specific force [m/$s^2$]')
    ax0.grid(True)
    ax0.legend()
    ax1 = plt.subplot(614, sharex=ax0)
    ax1.plot((cam_df['time'] - cam_df['time'][0]) * 1e-9, cam_df['y'], 'r', label='y')
    ax1.set_xlabel('t [sec]')
    ax1.set_ylabel(r'position [cm]')
    ax1.grid(True)
    ax1.legend()

    ax0 = plt.subplot(615)
    ax0.plot((imu_df['time'] - imu_df['time'][0]) * 1e-9, imu_df['z'], label='z')
    ax0.set_xlabel('t [sec]')
    ax0.set_ylabel(r'specific force [m/$s^2$]')
    ax0.grid(True)
    ax0.legend()
    ax1 = plt.subplot(616, sharex=ax0)
    ax1.plot((cam_df['time'] - cam_df['time'][0]) * 1e-9, cam_df['z'], 'r', label='z')
    ax1.set_xlabel('t [sec]')
    ax1.set_ylabel(r'position [cm]')
    ax1.grid(True)
    ax1.legend()

    fig1 = plt.figure()

    ax0 = plt.subplot(611)
    ax0.plot((imu_df['time']-imu_df['time'][0])*1e-9, imu_df['yaw'], label='yaw')
    ax0.set_xlabel('t [sec]')
    ax0.set_ylabel(r'angular vel [rad/s]')
    ax0.grid(True)
    ax0.legend()
    ax1 = plt.subplot(612, sharex=ax0)
    ax1.plot((cam_df['time'] - cam_df['time'][0]) * 1e-9, cam_df['yaw'], 'r', label='yaw')
    ax1.set_xlabel('t [sec]')
    ax1.set_ylabel(r'angle [deg]')
    ax1.grid(True)
    ax1.legend()

    ax0 = plt.subplot(613)
    ax0.plot((imu_df['time'] - imu_df['time'][0]) * 1e-9, imu_df['pitch'], label='pitch')
    ax0.set_xlabel('t [sec]')
    ax0.set_ylabel(r'angular vel [rad/s]')
    ax0.grid(True)
    ax0.legend()
    ax1 = plt.subplot(614, sharex=ax0)
    ax1.plot((cam_df['time'] - cam_df['time'][0]) * 1e-9, cam_df['pitch'], 'r', label='pitch')
    ax1.set_xlabel('t [sec]')
    ax1.set_ylabel(r'angle [deg]')
    ax1.grid(True)
    ax1.legend()

    ax0 = plt.subplot(615)
    ax0.plot((imu_df['time'] - imu_df['time'][0]) * 1e-9, imu_df['roll'], label='roll')
    ax0.set_xlabel('t [sec]')
    ax0.set_ylabel(r'angular vel [rad/s]')
    ax0.grid(True)
    ax0.legend()
    ax1 = plt.subplot(616, sharex=ax0)
    ax1.plot((cam_df['time'] - cam_df['time'][0]) * 1e-9, cam_df['roll'], 'r', label='roll')
    ax1.set_xlabel('t [sec]')
    ax1.set_ylabel(r'angle [deg]')
    ax1.grid(True)
    ax1.legend()

    plt.show()


if __name__ == '__main__':
    main()
