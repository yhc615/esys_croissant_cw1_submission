#Import the general libraries 
import machine
import time

#disable output debug
import esp
esp.osdebug(None)

#Configure network settings
import network

ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)

sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect('EEERover', 'exhibition')

time.sleep(1)

#Configuration steps for temperature + humidity sensor (Si7021)
#Import the libraries
from machine import Pin, I2C
from time import sleep_ms

# Default Address
SI7021_I2C_DEFAULT_ADDR = 0x40

# Commands
CMD_MEASURE_RELATIVE_HUMIDITY_HOLD_MASTER_MODE = 0xE5
CMD_MEASURE_RELATIVE_HUMIDITY = 0xF5
CMD_MEASURE_TEMPERATURE_HOLD_MASTER_MODE = 0xE3
CMD_MEASURE_TEMPERATURE = 0xF3
CMD_READ_TEMPERATURE_VALUE_FROM_PREVIOUS_RH_MEASUREMENT = 0xE0
CMD_RESET = 0xFE
CMD_WRITE_RH_T_USER_REGISTER_1 = 0xE6
CMD_READ_RH_T_USER_REGISTER_1 = 0xE7
CMD_WRITE_HEATER_CONTROL_REGISTER = 0x51
CMD_READ_HEATER_CONTROL_REGISTER = 0x11

#Class that contains functions which read the raw data from the sensor and then processes it to give values in degrees celsius (temperature) and percentages (relative humidity)
class Si7021(object):
    def __init__(self, i2c_addr = SI7021_I2C_DEFAULT_ADDR):
        self.addr = i2c_addr
        self.cbuffer = bytearray(2)
        self.cbuffer[1] = 0x00
        self.i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)

    def write_command(self, command_byte):
        self.cbuffer[0] = command_byte
        self.i2c.writeto(self.addr, self.cbuffer)

    def readTemp(self):
        self.write_command(CMD_MEASURE_TEMPERATURE)
        sleep_ms(25)
        temp = self.i2c.readfrom(self.addr,3)
        temp2 = temp[0] << 8
        temp2 = temp2 | temp[1]
        return (175.72 * temp2 / 65536) - 46.85

    def readRH(self):
        self.write_command(CMD_MEASURE_RELATIVE_HUMIDITY)
        sleep_ms(25)
        rh = self.i2c.readfrom(self.addr, 3)
        rh2 = rh[0] << 8
        rh2 = rh2 | rh[1]
        return (125 * rh2 / 65536) - 6

#Create objects of the class Si7021 to be used for retreiving + processing raw data from the temeprature and humidity sensor
Temperature = Si7021()
Humidity = Si7021()

#Configuration steps for colour sensor (TCS34725)
# Default Address
TCS34725_I2C_DEFAULT_ADDR = 0x29

#Command Registers: These addresses have 0x80 added to them which is the command byte (see enable register below as an example)
TCS34725_ENABLE = 0X80 #enbale register has address 0x00 but we add to it 0x80 to include the command byte. This applies to all the other definitions
TCS34725_CONFIGURATION = 0X8D
TCS34725_WAIT_TIME = 0x83
TCS34725_RGBC_TIME = 0x81
TCS34725_CONTROL = 0X8F
TCS34725_CLEAR_LOW_BYTE = 0X94 #Sensor uses 16 bits (2 bytes) to express RGBC values
TCS34725_CLEAR_HIGH_BYTE = 0X95
TCS34725_RED_LOW_BYTE = 0X96
TCS34725_RED_HIGH_BYTE = 0X97
TCS34725_GREEN_LOW_BYTE = 0X98
TCS34725_GREEN_HIGH_BYTE = 0X99
TCS34725_BLUE_LOW_BYTE = 0X9A
TCS34725_BLUE_HIGH_BYTE = 0X9B

class TCS34725(object):
	#Configure the sensor
    def __init__(self):
        self.i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
        self.i2c.writeto_mem(TCS34725_I2C_DEFAULT_ADDR,TCS34725_ENABLE,b'\x0B')#Enable register set to power on and activate wait time for power management
        self.i2c.writeto_mem(TCS34725_I2C_DEFAULT_ADDR,TCS34725_CONFIGURATION,b'\x02')#Configuration set to WLONG = 1
        self.i2c.writeto_mem(TCS34725_I2C_DEFAULT_ADDR,TCS34725_WAIT_TIME,b'\xAB')#Wait time register set to 2.45s
        self.i2c.writeto_mem(TCS34725_I2C_DEFAULT_ADDR,TCS34725_RGBC_TIME,b'\x00')#RGBC Timer register: set to max sensitivty 
        self.i2c.writeto_mem(TCS34725_I2C_DEFAULT_ADDR,TCS34725_CONTROL,b'\x00')#Control register: set to gain of 1
#Functions to read RGBC values
    def read_red(self):
        x = bytearray(self.i2c.readfrom_mem(TCS34725_I2C_DEFAULT_ADDR, TCS34725_RED_LOW_BYTE, 2))#Read 2 bytes to include the low byte and the high byte
        red = x[0] + x[1]*256
        return red

    def read_green(self):
        y = bytearray(self.i2c.readfrom_mem(TCS34725_I2C_DEFAULT_ADDR, TCS34725_GREEN_LOW_BYTE, 2))
        green = y[0] + y[1]*256
        return green

    def read_blue(self):
        z = bytearray(self.i2c.readfrom_mem(TCS34725_I2C_DEFAULT_ADDR, TCS34725_BLUE_LOW_BYTE, 2))
        blue = z[0] + z[1]*256
        return blue
    
    def read_clear(self):
        w = bytearray(self.i2c.readfrom_mem(TCS34725_I2C_DEFAULT_ADDR, TCS34725_CLEAR_LOW_BYTE, 2))
        clear_light = w[0] + w[1]*256 #w[1] is the high byte and w[0] is the low byte 
        return clear_light

#Create objects of the class TCS34725 to be used for retreiving + processing raw data from the colour sensor
Colour = TCS34725()

#Configuration of ujson message and connection to broker
import ujson
from umqtt.simple import MQTTClient
broker_address = "192.168.0.10"
c = MQTTClient("Embedded", broker_address)
message_received = False

#User Configuration settings: These are the target values (for temperature, humidity and lumniosity) that the room should be at
#Note that these values are initially defined here but in the next iteration will be defined by the user via the MQTT communciation set-up between the embedded system and the user's personal device
cycle_time = 5 #The amount of time (period in seconds) for which the user wants to receive data from the sensor
temp_target = 30 # Target temeprature for the storage room (in degrees Celsius)
temp_tolerance = 0.1 #The percentage value for which the user is allowing the current room temperature to deviate from the target temperature
humidity_target = 40 
humidity_tolerance = 0.5 
luminosity_target = 320000
luminosity_tolerance= 0.1

#Functions for temeprature, humidity and light intensity which return an ok message or error emssage depedning on the current values, target values and allowed tolerances. Note the latter two are user-defined after the first iteration
#Function to check if current values lie within the user defined boundaries
def bound_check(current,target,tolerance,mode):#mode = {0:'humidty',1:'temperature', 2:'luminosity'}
            lbound = target*(1 - tolerance)
            rbound = target*(1 + tolerance)
            if mode == 1 or mode == 0:
                if  current <= rbound and current >= lbound:
                    return "{}: good".format(current)
                else: 
                    if mode == 1:
                        return "Alert: Temperature is {}, not in the required range".format(current)
                    else:
                        return "Alert: Humidity is {}, not in the required range".format(current)  
            else:
                if current >= rbound:
                    return "Alert: Lux is {}, not in the required range".format(current)
                else:
                    return "{}: good".format(current)

#Function to calculate luminosity value                   
def calclux(red,green,blue):
        lux = 179.0 * (0.56*red + 0.65*green +  0.87*blue) 
        return lux

#Functions for subscribing to recevive a json message from the user containg the configuaration settings (target values, tolerances and cycle time)
def sub_cb(topic, msg):
    global jsonTopython
    jsonTopython = ujson.loads(msg)

def subscribed(message_received):
    c.set_callback(sub_cb)
    c.connect()
    c.subscribe(b"user_config")
    while message_received == False:
        if message_received == False:
            # Blocking wait for message
            c.wait_msg()
            message_received = True
        else:
            # Non-blocking wait for message
            c.check_msg()
            # Then need to sleep to avoid 100% CPU usage (in a real
            # app other useful actions would be performed instead)
            time.sleep(1)
    c.disconnect()
  
#Main loop
while True:
   #Read sensor data
   red = Colour.read_red()
   blue = Colour.read_blue()
   green = Colour.read_green()
   clear = Colour.read_clear()
   temp_current = Temperature.readTemp() 
   humidity_current = Humidity.readRH() 
   luminosity_current = calclux(red,green,blue)
    #Create and publish ujson message containing processed sensor data to MQTT broker 
   sensor_data = {
        'Temperature:' : bound_check(temp_current,temp_target,temp_tolerance,1),
        'Humidity' : bound_check(humidity_current,humidity_target,humidity_tolerance,0),
        'RGBC' : [red,green,blue,clear],
        'Luminosity' : bound_check(luminosity_current,luminosity_target,luminosity_tolerance,2)
        }
   payload = ujson.dumps(sensor_data)
   c.connect()
   c.publish(b"sensor_data", payload)
   c.disconnect()
       
   #Get ready to receive user configurations from MQTT broker 
   subscribed(message_received)
    
   #Update the thresholds depending on user configuration in the ujson message received from the user's personal device
   temp_target = jsonTopython["temperature target"]
   temp_tolerance = jsonTopython["temperature tolerance"]
   humidity_target = jsonTopython["humidity target"]
   humidity_tolerance = jsonTopython["humidity tolerance"]
   luminosity_target = jsonTopython["luminosity target"]
   luminosity_tolerance = jsonTopython["luminosity tolerance"]
   cycle_time = jsonTopython["cycle time"]
   time.sleep(cycle_time)

"""Please note that the time.sleep(cycle_time) should have been replaced with the deep sleep mode code which can be found below. However, our board had a hardware reset error that made this code not work despite the reset being connected to GPIO16:

# configure RTC.ALARM0 to be able to wake the device
rtc = machine.RTC()
rtc.irq(trigger=rtc.ALARM0, wake=machine.DEEPSLEEP)

# check if the device woke from a deep sleep
if machine.reset_cause() == machine.DEEPSLEEP_RESET:
    print('woke from a deep sleep')

# set RTC.ALARM0 to fire after 10 seconds (waking the device)
rtc.alarm(rtc.ALARM0, 10)

# put the device to sleep
machine.deepsleep()
print("hello")
"""



