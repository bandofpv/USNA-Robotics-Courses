import roslibpy
import numpy as np
import matplotlib.pyplot as plt
import time

class MapSubscriber:
    def __init__(self, id=86):
        self.id = id
        self.running = True
        
        # ROS variables
        self.client = roslibpy.Ros(host=f'10.24.6.{self.id}', port=9090)
        self.client.run()

        # Matplotlib figure for plotting and close handler
        self.fig = plt.figure()
        self.fig.canvas.mpl_connect('close_event', self.on_close)

        # Map variables
        self.occupancy_grid = None
        self.grid_size = None
        self.width = None
        self.height = None

        # ROS Subscriber for Occupancy Grid
        self.sub = roslibpy.Topic(self.client, f'/create_{self.id}/occupancy_grid', 'nav_msgs/OccupancyGrid')
        self.sub.subscribe(self.map_callback)

    def map_callback(self, msg):
        self.width = msg['info']['width']
        self.height = msg['info']['height']
        self.grid_size = msg['info']['resolution']
        
        # Convert the flattened list back into a 2D numpy array
        self.occupancy_grid = np.array(msg['data']).reshape((self.height, self.width))

    def on_close(self, event):
        print("\nPlot window closed. Stopping...")
        self.running = False
        try:
            self.client.terminate()
        except Exception:
            pass

    def plot_map(self):
        # Check if occupancy grid is available
        if self.occupancy_grid is None or self.grid_size is None:
            return
            
        plt.clf()
        x_half = (self.width * self.grid_size) / 2
        y_half = (self.height * self.grid_size) / 2
        
        plt.imshow(self.occupancy_grid, extent=[-x_half, x_half, -y_half, y_half], origin='lower', cmap='Greys')
        plt.xlim(-x_half, x_half)
        plt.ylim(-y_half, y_half)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title(f'Occupancy Grid')
        plt.grid()
        plt.pause(0.01)

if __name__ == "__main__":
    map_sub = MapSubscriber()
    print("Subscriber running. Press Ctrl+C to stop.")
    
    try:
        while map_sub.running:
            if plt.fignum_exists(map_sub.fig.number):
                map_sub.plot_map()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
        map_sub.client.terminate()
