import sys

import paho.mqtt.client as paho

client = paho.Client()

if client.connect("localhost", 1883, 60) != 0:
    print("Couldn't connect to the mqtt broker")
    sys.exit(1)

client.publish("sensors/max30102/data", "{\"ir\":123,\"red\":456,\"green\":789,\"finger\":true}", 0)
client.disconnect()