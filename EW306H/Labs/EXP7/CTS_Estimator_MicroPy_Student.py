from sbc import SBC
import time
import math
import machine

lab = SBC()  # Instantiate the RP2040 WSESBC

# setup the MAX1270 ADC
lab._init_adc()
lab._adc_device.bipolar = 0  # 0 unipolar, 1 bipolar; pressure sensors are unipolar
lab._adc_device.range = 0  # 0 5V range, 1 10V range; pressure sensors output voltage ~1-4 Volts
# lab._adc_device.read_volts(ch) # Returns reading in Volts, sample syntax

# motor port initialization (external to SBC)
#lab._init_mot1_dig()
lab._init_mot1_pwm()

# set up experiment parameters
Ts = 1.0 # sampling period for discrete-time matrices imported from MATLAB
streamPeriod = Ts # seconds

# pump data from experiment
PUMP_S = 0.009 #12V Pump Curve Slope (s/cm^3)
PUMP_I = 0.56 #12V Pump Curve Intercept (DC)
#dc = PUMP_S*FRin0+PUMP_I; # from (cm^3/s) to (DC)


#############################################
# *** Define Practical Operating Point  *** #
#############################################
# operating flow 
FRin0 = 20.00
h10=18.9 # cm
h20=18.9 # cm

# actual flow
FRin = 20.00 # same as FRin0

# sensor calibration parameters (SOURCE: EW301)
slope = 6.285 
vi = 1.948 # vertical intercept

#######################################
# *** Copy Matrices from MATLAB ***   #
#######################################
# Matrix PHI_obs values copied from MATLAB (T=1s) 
phi = [[0.932329722484034,-0.378771739905599], # row 0
       [0.067670277515966,0.787670277515967]] # row 1

# Matrix GAMMA_u values copied from MATLAB (T=1s) 
gamm = [0.064296909215259,0.0]

# Matrix G values copied from MATLAB (independent of T)
G = [0.378771739905599,0.144659444968067]

# Initializations
#result=[[0] * no. of cols] * no. of rows
X=[[0] * 2] * 500 # x_hat
Y=[[0] * 1] * 500 # y
U=[[0] * 1] * 500 # u
Yhat=0.0          # y_hat

def main(): # main function comprising experiment sequence
    z=1
    while z!=0:
        lab._mot1.set_w(0.0) # turn motor off
        
        t_final = 200 # up to 200 seconds

        t_start = time.ticks_ms()  # clock time at start of experiment
        t_elapsed = 0.0 # variable to represent the elapsed time of the experiment
        streamSample = 0.0
        index = 0 # to X, Y, U arrays

        while t_elapsed < t_final:

            #t_elapsed = time.monotonic() - t_start # measure time since start of experiment
            t_elapsed = (time.ticks_ms() - t_start)*10**(-3) # millisecond to second
            # read from both pressure sensors
            tank1 = lab._adc_device.read_volts(1) # Returns reading in Volts from channel 1 (tank 1)
            tank2 = lab._adc_device.read_volts(0) # Returns reading in Volts from channel 2 (tank 2)

            h1 = slope*tank1 + vi # height of tank 1
            h2 = slope*tank2 + vi # height of tank 2

            #######################################
            # ***START OBSERVER CODE***           #
            #######################################

            #DELTA flow rate (i.e., difference from OP)
            delta_q = FRin - FRin0
            
            #DELTA measured height in Tank 2 (i.e., difference from OP)
            delta_h2 = h2 - h20
             
            #DELTA estimated output at "nT" (i.e., output equation)
            YHat = X[index][1] 
            
            #DELTA estimated state at "(n+1)T" (i.e., sampled-data state equation)
            X[index+1][0] = phi[0][0]*X[index][0] + phi[0][1]*X[index][1] + gamm[0]*delta_q + G[0]*delta_h2
            X[index+1][1] = phi[1][0]*X[index][0] + phi[1][1]*X[index][1] + G[1]*delta_h2
            
            #######################################
            # ***END OBSERVER CODE***             #
            #######################################

            # convert voltage to duty cycle
            dc = PUMP_S*FRin+PUMP_I
            
            if dc > 1: # limit duty cycle between [0,1]
                dc = 1
            if dc < -1:
                dc = 0
            
            # send motor command to motor driver
            lab._mot1.set_w(dc)
            # 
            time.sleep(Ts) # seconds
            if t_elapsed > streamSample:
                streamSample = t_elapsed + streamPeriod
                out_msg = f"{t_elapsed},{X[index][0]+h10},{h1},{X[index][1]+h20},{h2},{dc}"
                index=index+1
                print(out_msg) # to REPL
        print("End of the experiment.") #
        lab._mot1.set_w(0.0) # turn motor off
        z=0

 
main()  # run the primary main method