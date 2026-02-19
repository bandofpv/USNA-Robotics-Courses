import time
import numpy as np
import matplotlib.pyplot as plt
from create3_sim import CreateSim

create = CreateSim(name='create')

if __name__ == '__main__':
    create.start()
    time.sleep(1.0)

    t_start = create.get_sim_time()
    create.update()
    while create.get_sim_time() - t_start < 10.0:
        # print(f"Sim Time: {create.get_sim_time():.2f} seconds")
        create.set_cmd_vel(u=0.2, r=0.5)
        create.update()

        scan_data = create.get_scan_data()

        
        
        # print(f"Scan Data: {scan_data}")
        n = len(scan_data)
        
        scan_data = np.array(scan_data)
        
        angle_min = -120.0*3.14159/180.0
        angle_max = 120.0*3.14159/180.0
        angles = np.linspace(angle_min, angle_max, n)

        ind = scan_data < 5.0
        scan_data = scan_data[ind]
        x = scan_data * np.cos(angles[ind])
        y = scan_data * np.sin(angles[ind])

        plt.clf()
        plt.plot(x, y, 'b.')
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        plt.pause(0.01)

        # print(f"Scan Data Length: {len(scan_data) if scan_data is not None else 0}")
        # time.sleep(0.05)

    # create.set_cmd_vel(u=0.0, r=0.0)
    # create.update()
    time.sleep(1.0)

    create.stop()