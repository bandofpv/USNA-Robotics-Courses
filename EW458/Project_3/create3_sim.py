# !/usr/bin/env python3

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

import numpy as np
import json

def wrapToPi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class CreateSim():
    def __init__(self, name='create'):

        self.name = name

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.sim.setStepping(True)
        self.sim_status = 'stopped'

        if(self.sim.getSimulationState() == self.sim.simulation_advancing_running):
            self.sim.stopSimulation()
            time.sleep(0.5)

        self.u_cmd = 0.0
        self.r_cmd = 0.0
        self.scan_data = None

        self.odom = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.pose = (0.0, 0.0, 0.0)

        self.get_handles()


    def get_handles(self):
        self.create_h = self.sim.getObjectHandle(self.name)
        self.odom_frame_h = self.sim.getObjectHandle('/odom_frame')
        # self.imu_h = self.sim.getObjectHandle(':/imu')
        # self.camera_h = self.sim.getObjectHandle(':/oak_sensor')
        # self.oak_h = self.sim.getObjectHandle(':/oak_d')

    def set_cmd_vel(self, u=0.0, r=0.0):
        self.u_cmd = u
        self.r_cmd = r
    
    def get_scan_data(self):
        return self.scan_data


    def start(self):
        self.sim.startSimulation()

    def stop(self):
        self.sim.stopSimulation()

    def get_odom(self):
        return self.odom
    
    def get_pose(self):
        return self.pose

    def reset_odom(self):
        pose = self.sim.getObjectPose(self.create_h, -1)
        self.sim.setObjectPose(self.odom_frame_h, -1, pose)
        # Add code to reset odometry in CoppeliaSim here
        pass    

    def get_sim_time(self):
        return self.sim.getSimulationTime()

    def update(self):

        sim_state = self.sim.getSimulationState()
        # print(f"Simulation State: {sim_state} = {self.sim.simulation_advancing_running}")
        if(sim_state == self.sim.simulation_advancing_running):

            pos = self.sim.getObjectPosition(self.create_h, -1)
            eul = self.sim.getObjectOrientation(self.create_h, -1)
            self.pose = (pos[0], pos[1], eul[2])
            # print(f"Pose: {self.pose}")
            
            self.sim.setFloatSignal(self.name+'_u', self.u_cmd)
            self.sim.setFloatSignal(self.name+'_r', self.r_cmd)


            x = self.sim.getFloatSignal(self.name+'_odom_x')
            y = self.sim.getFloatSignal(self.name+'_odom_y')
            theta = self.sim.getFloatSignal(self.name+'_odom_theta')
            vx = self.sim.getFloatSignal(self.name+'_odom_vx')
            vy = self.sim.getFloatSignal(self.name+'_odom_vy')
            r = self.sim.getFloatSignal(self.name+'_odom_r')
            self.odom = (x, y, theta, vx, vy, r)


            myData = self.sim.getBufferProperty(self.create_h, "customData.scanData", {'noError' : True})
            if myData:
                myData = self.sim.unpackTable(myData)
                self.scan_data = myData

        self.sim.step()


