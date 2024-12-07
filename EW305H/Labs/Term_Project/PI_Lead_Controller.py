from sbc import SBC
import time
import math
import ttyacm
import gc

lab = SBC() # Instantiate the RP2040 WSESBC

# set up experiment parameters
run_time = 10.0 # seconds
updateRate = 100 # Hz
T = 1/updateRate # sampling period in sec
z_PI = 4.4538 # PI zero for G_c(s) TF
z_c = 3.3751 # Lead zero for G_c(s) TF
p_c = 27.2156 # Lead pole for G_c(s) TF
K = 2.3835 # proportional gain
ref_pos = math.pi # rad

def main(): # main fucntion comprising experiment sequence
    output_prev = 0 # initialize previous output voltage
    intermediate_prev = 0 # initialize previous intermeidate voltage 
    error_prev = 0 # initialize previous error variable

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
        while t_elapsed < run_time:
            t_elapsed = (time.ticks_ms() - t_start)*10**(-3) #sec
            # measure time since start of experiment
            count = lab._enc_device1.read_counter() # read from encoder
            pos = count*2 *math.pi/4096
            speed = (pos - pos_prev)/T # estimate speed based on change in
            pos_prev = pos
            
            # implement proportional integral controller
            error = ref_pos - pos
            
            # Discretized PI and Lead Controllers
            PI_Controller_Voltage = intermediate_prev + (K/2.0)*((2.0+z_PI*T)*error + (z_PI*T - 2.0)*error_prev)
            intermediateVoltage = PI_Controller_Voltage # intermediary output voltage from PI controller
            Lead_Controller_Voltage = -1*((-2+p_c*T)/(2+p_c*T))*output_prev + ((2+z_c*T)/(2+p_c*T))*intermediateVoltage + ((-2+z_c*T)/(2+p_c*T))*intermediate_prev
#             Lead_Controller_Voltage = -1*((-2+p_c*T)/(2+p_c*T))*output_prev + ((2*K+K*z_c*T)/(2+p_c*T))*intermediateVoltage + ((-2*K+K*z_c*T)/(2+p_c*T))*intermediate_prev
            outputVoltage = Lead_Controller_Voltage 
            
            # update prev variables
            output_prev = outputVoltage
            intermediate_prev = intermediateVoltage
            error_prev = error
            
            if outputVoltage > 10: #soft limits on motor voltage [-10,10]
                outputVoltage = 10
            if outputVoltage < -10:
                outputVoltage = -10
                        
            #send motor command to digital potentiometer
            lab._digipot_device.set_volt(outputVoltage)
            
            #store data in array
#             t_list.append(t_elapsed)
#             pos_list.append(pos)
#             spd_list.append(speed)
#             inpt_list.append(outputVoltage)
#             error_list.append(error)
            
            out_msg = f"{t_elapsed},{pos},{speed},{outputVoltage},{error};"
            tty.print(out_msg)
#             print(out_msg)
            
            #countloop += 1
            time.sleep(T) #wait until sampling period ends
            
            
        print("Finished trial") # confirm
        lab._digipot_device.set_pot(0) # turn off motor
        
        #send data back to MATLAB
#         for i in range(len(t_list)):
#             out_msg = f"{t_list[i]},{pos_list[i]},{spd_list[i]},{inpt_list[i]},{error_list[i]};"
#             tty.print(out_msg)
#             print(out_msg)
#         print("finished sending trial data")
        if n == 1:
            break 
        
main()
