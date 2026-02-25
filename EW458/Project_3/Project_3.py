import time
import math
from matplotlib.path import Path
import numpy as np
from scipy import ndimage
from RRT import RRT
import matplotlib.pyplot as plt
from create3_sim import CreateSim

# Utility function to wrap angles to [-pi, pi]
def wrapToPi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# Handle window close event to stop simulation loop
def on_close(event):
    global sim_running
    print("Window closed. Exiting simulation...")
    sim_running = False

create = CreateSim()

occupancy_grid = np.zeros((100, 100))
grid_size = 0.1  # Size of each grid cell
x_range = np.round(np.linspace(-5.0, 5.0, len(occupancy_grid)),decimals=1)
y_range = np.round(np.linspace(-5.0, 5.0, len(occupancy_grid)),decimals=1)
x_grid, y_grid = np.meshgrid(x_range, y_range)

robot_radius = 0.3  # robot radius in meters
inflation_radius = int(np.ceil(robot_radius / grid_size))  # convert to grid cells

# Define goal point
goal_point = [-4.0, 2.0] 

# Simulation state variables
sim_running = True
wp_radius = 0.1 # radius to consider waypoint reached
des_path = None
u = 0.0
r = 0.0

# Initialize variables
wp_index = 1
robot_path = [] # for visualization of robot trajectory
rrt_tree_nodes = [] # for visualization of RRT tree nodes

# Define map parameters for RRT
map_params_config = {
    'res': grid_size,
    'x_limit': [-5, 5],
    'y_limit': [-5, 5],
    'origin': [-5, -5] 
}

if __name__ == '__main__':
    create.start()
    time.sleep(1.0)
    
    # Initialize plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(6,6))
    
    # Connect the close event
    fig.canvas.mpl_connect('close_event', on_close)

    # Build Occupancy Grid from Walls
    for i in range(21):
        ob_name = f'/wall[{i}]'
        wall_h = create.sim.getObjectHandle(ob_name)
        pos = create.sim.getObjectPosition(wall_h)
        eul = create.sim.getObjectOrientation(wall_h)
        res,type,dim = create.sim.getShapeGeomInfo(wall_h)
        
        # Define the corners of the wall in its local frame
        pts_b = np.array([[dim[0]/2, -dim[0]/2, -dim[0]/2, dim[0]/2],
                        [dim[1]/2, dim[1]/2, -dim[1]/2, -dim[1]/2]])

        # Rotation matrix from body frame to global frame
        R_bg = np.array([[np.cos(eul[2]),-np.sin(eul[2])],[np.sin(eul[2]),np.cos(eul[2])]])

        # Transform points to global frame
        pts_g = R_bg @ pts_b + np.array([[pos[0],pos[0],pos[0],pos[0]],
                                        [pos[1],pos[1],pos[1],pos[1]]])

        path = Path(pts_g.T)

        # Fill occupancy grid cells that are inside the wall polygon
        contains = path.contains_points(np.vstack((x_grid.flatten(), y_grid.flatten())).T)
        occupancy_grid[contains.reshape(x_grid.shape)] = 255

    # Inflate the occupied cells to create a buffer around obstacles
    occupancy_grid = ndimage.binary_dilation(occupancy_grid, iterations=inflation_radius).astype(occupancy_grid.dtype)

    t_start = create.get_sim_time()
    create.update()

    # Controller Gains
    Kp_yaw = 0.5
    Kp_speed = 2.0
    
    # Run simulation loop
    while sim_running: 
        t = create.get_sim_time()

        # Run RRT Planning
        if des_path is None:
             # Stop while planning
            create.set_cmd_vel(0,0)
            
            # Plan only if figure is open and enough time has passed to accumulate map data
            if plt.fignum_exists(fig.number) and (t - t_start > 1.0):
                start_pos = [create.pose[0], create.pose[1]]
                print("Planning path...")
                planner = RRT(start=start_pos, 
                              goal=goal_point, 
                              map_grid=occupancy_grid, 
                              map_params=map_params_config, 
                              expand_dis=0.5, 
                              max_iter=500) 
                
                path_result = planner.plan()
                rrt_tree_nodes = np.array(planner.tree)
                
                if path_result is not None:
                    des_path = np.array(path_result)
                    print(f"Path found with {len(des_path)} waypoints")
        
        # Control Loop
        
        if des_path is not None:
            if wp_index < len(des_path):
                # Get current target waypoint
                current_target = des_path[wp_index]
                
                # Calculate distance to target
                dx = current_target[0] - create.pose[0]
                dy = current_target[1] - create.pose[1]
                dist = math.sqrt(dx**2 + dy**2)
                
                # Speed control
                u_des = Kp_speed * dist
                u = max(0, min(u_des, 0.306)) # cap forward speed 

                # Calculate desired heading and heading error
                psi_des = math.atan2(dy, dx)
                psi_err = wrapToPi(psi_des - create.pose[2])
                
                # Heading controller
                r_des = Kp_yaw * psi_err
                r = max(-1.57, min(r_des, 1.57)) # cap turn rate

                # Check if reached waypoint
                if dist < wp_radius:
                    wp_index += 1
                
                # Slow down if heading error is large
                if abs(psi_err) > math.radians(20):
                    u = 0.0
                        
            else:
                # print("Goal Reached!")
                u = 0.0
                r = 0.0
                # des_path = None 
            
        create.set_cmd_vel(u, r)
        create.update()

        robot_path.append((create.pose[0], create.pose[1]))

        ax.clear()
        
        # Plot Occupancy Grid
        ax.imshow(occupancy_grid, extent=[-5, 5, -5, 5], origin='lower', cmap='gray', alpha=0.6)
        
        # Plot RRT Tree
        if len(rrt_tree_nodes) > 0:
            ax.plot(rrt_tree_nodes[:, 0], rrt_tree_nodes[:, 1], 'c.', markersize=2)  

        # Plot Goal
        ax.plot(goal_point[0], goal_point[1], 'g*', markersize=15)

        # Plot Active Path
        if des_path is not None:
            ax.plot(des_path[:, 0], des_path[:, 1], 'g-', linewidth=2)

            # Highlight current target waypoint
            if wp_index < len(des_path):
                ax.plot(des_path[wp_index, 0], des_path[wp_index, 1], 'y.', markersize=10)

        # Plot Robot
        ax.plot(create.pose[0], create.pose[1], 'ro')
        ax.arrow(create.pose[0], create.pose[1], 0.5*np.cos(create.pose[2]), 0.5*np.sin(create.pose[2]), 
                    head_width=0.2, head_length=0.3, fc='r', ec='r')

        # Plot Robot Path
        if len(robot_path) > 1:
            path_array = np.array(robot_path)
            ax.plot(path_array[:, 0], path_array[:, 1], 'r-')

        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_title(f"Sim Time: {t:.2f}s")
        plt.draw()
        plt.pause(0.001)

    time.sleep(1.0)
    create.stop()
    plt.close('all')