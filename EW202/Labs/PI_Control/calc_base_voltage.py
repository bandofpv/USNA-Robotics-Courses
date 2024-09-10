from machine import Pin, PWM, ADC
import time

# conversion constants
c = 1.246156257391686e-08
e = -0.001021188865621
f = 24.317200852875370

sensor = ADC(Pin(26)) # create ADC object for Pin 26
motor = PWM(Pin(16)) # DC motor control from GPIO16
motor.freq(1000) # set frequency to 1KHz -- DO NOT CHANGE
motor.duty_u16(0) # 0-65535 for duty cycle range 0-100

# read 3 values and return the minimum
def sample_sensor():
    val0 = sensor.read_u16()
    time.sleep_us(1500) #IMPORTANT time.sleep(float--> rounded to nearest ms)
    val1 = sensor.read_u16()
    time.sleep_us(1500)
    val2 = sensor.read_u16()
    return min([val0,val1,val2])

# calculate height given ADC voltage
def calc_height(voltage):
    hole = c*(voltage**2) + e*voltage + f # calculate hole given ADC voltage
    return (14-hole)*2

duty_cycle = 0.75 # set duty cycle
convert_dc = int(duty_cycle * 65535) # convert duty cycle to an integer 0 to 65535
motor.duty_u16(convert_dc) # set motor pin to new duty cycle

time.sleep(10)

samples = 100
data = list(range(samples))

for i in data:
    voltage = sample_sensor() # take a voltage reading
    height = calc_height(voltage) # convert voltage reading to height
    data[i] = height
    print(height)
    
    time.sleep(0.02)
    
avg = sum(data) / len(data)
print(f"Average: {avg}")
motor.duty_u16(0)