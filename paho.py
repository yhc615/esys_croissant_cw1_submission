#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  9 09:55:51 2018

@author: mohyeldinaboualam
"""

import paho.mqtt.client as mqttClient
import time
import json

#Preparing the user configurations (these can be modified by the user and are then sent to the device through the MQTT Broker)
user_input = {
        'temperature target' : 25,
        'temperature tolerance' : 0.1,
        'humidity target' : 30,
        'humidity tolerance' : 0.05,
        'luminosity tolerance': 0.1,
        'luminosity target': 320000,
        'cycle time' : 5
        }
user_data = json.dumps(user_input)

def on_connect(client, userdata, flags, rc):
 
    if rc == 0:
 
        print("Connected to broker")
 
        global Connected                #Use global variable
        Connected = True                #Signal connection 
        
 
    else:
 
        print("Connection failed")
 
def on_message(client, userdata, message):
    print("Message received: ", str(message.payload.decode("utf-8")))
    client.publish(topic = "user_config", payload = user_data) #send user data (in ujson message format) whenever we receive data from the device
    time.sleep(1)
 
Connected = False   #global variable for the state of the connection
 
broker_address= "192.168.0.10"  #Broker address

client = mqttClient.Client("Mac") 
client.on_connect= on_connect                      #attach function to callback
client.on_message= on_message                      #attach function to callback
 
client.connect(broker_address)          #connect to broker
 
client.loop_start()        #start the loop
 
while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
client.subscribe("sensor_data") 
try:
    while True:
        time.sleep(1)
 
except KeyboardInterrupt:
    print ("exiting")
    client.disconnect()
    client.loop_stop()
