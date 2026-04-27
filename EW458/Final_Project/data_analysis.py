import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
import os

def wrapToPi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def read_data(pose_file, des_pose_file, occupancy_grid_file, occupancy_grid_info_file):
    pose_data = []
    des_pose_data = []
    occupancy_grid = np.array([])
    occupancy_grid_info = {}

    # Read pose data
    try:
        # Skip header explicitly, or use pandas/numpy
        pose_data = np.genfromtxt(pose_file, delimiter=',', skip_header=1)
    except Exception as e:
        print(f"Error reading pose data: {e}")

    # Read desired pose data
    try:
        des_pose_data = np.genfromtxt(des_pose_file, delimiter=',', skip_header=1)
    except Exception as e:
        print(f"Error reading desired pose data: {e}")

    # Read occupancy grid data
    try:
        occupancy_grid = np.load(occupancy_grid_file)
    except Exception as e:
        print(f"Error reading occupancy grid data: {e}")

    # Read occupancy grid info
    try:
        occupancy_grid_info = np.load(occupancy_grid_info_file, allow_pickle=True).item()
    except Exception as e:
        print(f"Error reading occupancy grid info: {e}")

    return pose_data, des_pose_data, occupancy_grid, occupancy_grid_info

def compute_closest_path(pose_arr, occupancy_grid, occupancy_grid_info):
    if len(pose_arr) > 0 and occupancy_grid.size > 0:
        x_min = occupancy_grid_info['map_x_min']
        y_min = occupancy_grid_info['map_y_min']
        grid_size = occupancy_grid_info['grid_size']

        # Find coordinates of all obstacle cells (assume 1.0 represents an obstacle)
        obs_y_idx, obs_x_idx = np.where(occupancy_grid == 1.0)
        obs_x = obs_x_idx * grid_size + x_min
        obs_y = obs_y_idx * grid_size + y_min
        obstacles = np.column_stack((obs_x, obs_y))

        if len(obstacles) == 0:
            print("No obstacles found in occupancy grid.")
            return

        # Calculate distances from each robot position to all obstacles
        distances = cdist(pose_arr[:, :2], obstacles, metric='euclidean')
        min_distance = np.min(distances)
        
        print(f"Minimum distance to an obstacle: {min_distance:.3f} m")
        return min_distance

def plot_results(pose_arr, des_pose_arr, occupancy_grid, occupancy_grid_info):
    if len(pose_arr) > 0 and len(des_pose_arr) > 0:
        sample_count = min(len(pose_arr), len(des_pose_arr))
        pose_arr = pose_arr[:sample_count]
        des_pose_arr = des_pose_arr[:sample_count]
        time_steps = np.arange(sample_count)
        position_tol = 0.15
        yaw_tol_deg = 5.0

        posistion_error = cdist(pose_arr[:, :2], des_pose_arr[:, :2], metric='euclidean').diagonal()
        yaw_error = np.abs(wrapToPi(pose_arr[:, 2] - des_pose_arr[:, 2]))

        print("\nPlotting results...")

        output_dir = os.path.join('Final_Project', 'plots')
        os.makedirs(output_dir, exist_ok=True)

        plt.figure(figsize=(9, 6))
        plt.imshow(occupancy_grid, origin='lower', extent=[occupancy_grid_info['map_x_min'], occupancy_grid_info['map_x_max'], occupancy_grid_info['map_y_min'], occupancy_grid_info['map_y_max']], cmap='gray', vmin=0.0, vmax=1.0)
        plt.plot(pose_arr[:, 0], pose_arr[:, 1], 'r-', label='Robot Path')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Robot Path on Occupancy Grid')   
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(os.path.join(output_dir, 'robot_path_on_grid.png'), dpi=300, bbox_inches='tight')

        plt.figure(figsize=(9, 6))
        plt.plot(des_pose_arr[:, 0], des_pose_arr[:, 1], 'go', label='Desired Path')
        plt.plot(pose_arr[:, 0], pose_arr[:, 1], 'r-', label='Robot Path')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Robot vs Desired Path')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(os.path.join(output_dir, 'robot_vs_desired.png'), dpi=300, bbox_inches='tight')

        plt.figure(figsize=(10, 7))
        plt.subplot(2, 1, 1)
        plt.plot(time_steps, des_pose_arr[:, 0], '--', color='tab:green', label='Desired X')
        plt.plot(time_steps, des_pose_arr[:, 0] + position_tol, ':', color='tab:orange', linewidth=1.2, label='X Tolerance')
        plt.plot(time_steps, des_pose_arr[:, 0] - position_tol, ':', color='tab:orange', linewidth=1.2)
        plt.plot(time_steps, pose_arr[:, 0], '-', color='tab:red', label='Actual X')
        plt.xlabel('Time Step')
        plt.ylabel('X (m)')
        plt.title('X Position Over Time')
        plt.legend()
        plt.grid(True)

        plt.subplot(2, 1, 2)
        plt.plot(time_steps, des_pose_arr[:, 1], '--', color='tab:green', label='Desired Y')
        plt.plot(time_steps, des_pose_arr[:, 1] + position_tol, ':', color='tab:orange', linewidth=1.2, label='Y Tolerance')
        plt.plot(time_steps, des_pose_arr[:, 1] - position_tol, ':', color='tab:orange', linewidth=1.2)
        plt.plot(time_steps, pose_arr[:, 1], '-', color='tab:red', label='Actual Y')
        plt.xlabel('Time Step')
        plt.ylabel('Y (m)')
        plt.title('Y Position Over Time')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'position_over_time.png'), dpi=300, bbox_inches='tight')

        plt.figure(figsize=(10, 5))
        desired_yaw_deg = np.degrees(des_pose_arr[:, 2])
        plt.plot(time_steps, desired_yaw_deg, '--', color='tab:green', label='Desired Yaw')
        plt.plot(time_steps, desired_yaw_deg + yaw_tol_deg, ':', color='tab:orange', linewidth=1.2, label='Yaw Tolerance')
        plt.plot(time_steps, desired_yaw_deg - yaw_tol_deg, ':', color='tab:orange', linewidth=1.2)
        plt.plot(time_steps, np.degrees(pose_arr[:, 2]), '-', color='tab:red', label='Actual Yaw')
        plt.xlabel('Time Step')
        plt.ylabel('Yaw (degrees)')
        plt.title('Yaw Over Time')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'yaw_over_time.png'), dpi=300, bbox_inches='tight')

        plt.figure(figsize=(9, 6))
        plt.subplot(2, 1, 1)
        plt.plot(posistion_error, 'b-', label='Position Error (m)')
        plt.xlabel('Time Step')
        plt.ylabel('Position Error (m)')
        plt.title('Position Error Over Time')
        plt.legend()
        plt.grid(True)
        plt.subplot(2, 1, 2)
        plt.plot(np.degrees(yaw_error), 'm-', label='Yaw Error (degrees)')
        plt.xlabel('Time Step')
        plt.ylabel('Yaw Error (degrees)')
        plt.title('Yaw Error Over Time')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'errors.png'), dpi=300, bbox_inches='tight')

        plt.show()

if __name__ == "__main__":
    pose_arr, des_pose_arr, occupancy_grid, occupancy_grid_info = read_data("Final_Project/pose_data.csv", "Final_Project/des_pose_data.csv", "Final_Project/occupancy_grid.npy", "Final_Project/occupancy_grid_info.npy")
    compute_closest_path(pose_arr, occupancy_grid, occupancy_grid_info)
    plot_results(pose_arr, des_pose_arr, occupancy_grid, occupancy_grid_info)
