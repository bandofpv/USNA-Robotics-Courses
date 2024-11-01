from sbc import SBC
import time
import math
import ttyacm
import gc

lab = SBC() # Instantiate the RP2040 WSESBC

# set up experiment parameters
updateRate = 100 # Hz
per = 1/updateRate # sampling period in sec
kP = 0.1762 # proportional gain
ref_speed = 45 # rad/s

def main(): # main fucntion comprising experiment sequence
    tty = ttyacm.open(1) # open data port
    n = 0
    while 1: # run indefinitely
        n = n + 1 # break trigger
        gc.collect() # enable automatic [memeory] garbage collection
        
        motorVolt = 3.0
        
        t_list = [] # initialize array/list of time measurements
        pos_list = [] # initialize array/list of positon measurements
        spd_list = [] # initialize array/list of speed measurements
        inpt_list = [] # initialize array/list of motor input measurements
        error_list = [] # initialize array/list of error calculations
        pos_prev = lab._enc_device1.read_counter() * 2 * math.pi / 4096 # convert
        
        t_elapsed = 0.0 # variable to represent the elapsed time of the expirement
        t_start = time.ticks_ms() # clock time at start of experiment
        # Returns and increasing millisecond counter with an abritrary reference parameter
        # that wraps aroudn after some value.
        while t_elapsed < 1.0:
            t_elapsed = (time.ticks_ms() - t_start)*10**(-3) #sec
            # measure time since start of experiment
            count = lab._enc_device1.read_counter() # read from encoder
            pos = count*2 *math.pi/4096
            speed = (pos - pos_prev)/per # estimate speed based on change in __
            pos_prev = pos
            
            # implement proportional controller
            error = ref_speed - speed
            outputVoltage = kP * error 
            if outputVoltage > 10: #soft limits on motor voltage [-10,10]
                outputVoltage = 10
            if outputVoltage < -10:
                outputVoltage = -10
            
            #send motor command to digital potentiometer
            lab._digipot_device.set_volt(outputVoltage)
            
            #store data in array
            t_list.append(t_elapsed)
            pos_list.append(pos)
            spd_list.append(speed)
            inpt_list.append(outputVoltage)
            error_list.append(error)
            #countloop += 1
            time.sleep(per) #wait until sampling period ends
            
            
        print("Finished trial") # confirm
        lab._digipot_device.set_pot(0) # turn off motor
        
        #send data back to MATLAB
        for i in range(len(t_list)):
            out_msg = f"{t_list[i]},{pos_list[i]},{spd_list[i]},{inpt_list[i]},{error_list[i]};"
            tty.print(out_msg)
            print(out_msg)
        print("finished sending trial data")
        if n == 1:
            break 
        
main()
