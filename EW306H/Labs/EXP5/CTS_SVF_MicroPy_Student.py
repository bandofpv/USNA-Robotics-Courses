from sbc import SBC
import time
import math
import machine

lab = SBC()  # Instantiate the RP2040 WSESBC

# setup the MAX1270 ADC
lab._init_adc()
# motor port initialization (external to SBC)
lab._init_mot1_pwm()

# set up experiment parameters
streamPeriod = 1.0 # seconds

# pump data from experiment
PUMP_S = 0.009 #12V Pump Curve Slope (s/cm^3)
PUMP_I = 0.56 #12V Pump Curve Intercept (DC)
#dc = PUMP_S*FRin0+PUMP_I; # from (cm^3/s) to (DC)

# sensor calibration parameters
slope = 6.285 
vi = 1.948 # vertical intercept

# operating flow ***THIS IS AN EXAMPLE***
FRin0 = 20.00  # dc=0.9 & h10=17cm, h20=19cm 
h10 = 18.5 # cm
h20 = 18 # cm

# Declare the feedback gains ***THIS IS AN EXAMPLE***
K1 = 1.07 # for 15cm 
K2 = 0.23 # NOTE: changes w/ OP

def main(): # main function comprising experiment sequence
    z=1 # set flag high
    while z!=0:

        lab._mot1.set_w(0.0) # turn motor off
        t_final = 200 # 180 nominally

        t_start = time.ticks_ms()  # clock time at start of experiment
        t_elapsed = 0.0 # variable to represent the elapsed time of the experiment
        streamSample = 0.0

        while t_elapsed < t_final:

            t_elapsed = (time.ticks_ms() - t_start)*10**(-3) # sec
            # read from both pressure sensors
            tank1 = lab._adc_device.read_volts(1) # Returns reading in Volts from channel 1 (tank 1)
            tank2 = lab._adc_device.read_volts(0) # Returns reading in Volts from channel 2 (tank 2)
            h1 = slope*tank1 + vi # height of tank 1
            h2 = slope*tank2 + vi # height of tank 2

            # START SVF CONTROLLER RELATIONSHIPS
            delta_h1 = h1 - h10
            delta_h2 = h2 - h20
            delta_qi = -(K1*delta_h1 + K2*delta_h2)
            FRin = delta_qi + FRin0
            # END SVF CONTROLLER RELATIONSHIPS

            # convert voltage to duty cycle
            dc = PUMP_S*FRin+PUMP_I # ***NEED FRin FROM SVF ABOVE
#             dc = 0.74 # ***THIS IS AN EXAMPLE***

            if dc > 1: # limit duty cycle between [0,1]
                dc = 1
            if dc < -1:
                dc = 0

            # send motor command to motor driver
            lab._mot1.set_w(dc)
# 
            time.sleep(0.1)
            #print(t_elapsed)
            if t_elapsed > streamSample:
                streamSample = t_elapsed + streamPeriod
                out_msg = f"{t_elapsed},{h1},{h2},{dc}\n"
                print(out_msg) # to REPL
        
        print("End of the experiment.") #
        lab._mot1.set_w(0.0) # turn motor off
        z=0 # set flag low

 
main()  # run the primary main method