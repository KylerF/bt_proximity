"""
Bluetooth device proximity tracking.
Monitors the RSSI received from bluetooth devices and 
sends to HomeAssistant's Mosquitto instance.
"""
from bt_proximity import BluetoothRSSI
import paho.mqtt.client as mqtt
import socket
import json
import time
import sys

# TODO Move to a database!!!
BT_ADDR = 'C4:98:80:35:B6:1D'

# MQTT client connection details
broker = '192.168.1.5'
client_name = socket.gethostname()
client = mqtt.Client(client_name)
client.username_pw_set("kyler", "*()IOP890iop")

# MQTT publish details
client_topic = "pressence/{}/kyler".format(client_name)
publish_interval = 1
print(client_topic)

# RSSI averaging params
num_rssi_samples = 10  # Number of samples collected
rssi_delay       = 0.5 # Delay between measurements (sec)
connect_timeout  = 2   # BT connection timeout (sec)

# MQTT connection callback
def on_connect(client, userdata, flags, rc):
    print("Connected flags " + str(flags) + ", result code "+str(rc)+", client1_id ")

# MQTT data published callback
def on_publish(client, userdata, result):
    print("data published \n")

client.on_connect = on_connect
client.on_publish = on_publish

def main():
    # Connect to HomeAssistant
    client.connect(broker, 1883, 60)
    client.loop_start()

    # Init RSSI samples array
    rssi_samples = [-99] * num_rssi_samples
    sample_num = 0

    while True:
        # Attempt connection to device and query RSSI
        btrssi = BluetoothRSSI(addr=BT_ADDR)
        #btrssi.bt_sock.settimeout(connect_timeout)
        rssi = btrssi.get_rssi()

        # Default "away" value
        if not btrssi.connected:
            rssi = -99
        
        # Calculate the running mean
        rssi_samples[sample_num] = rssi
        sample_num = (sample_num + 1) % num_rssi_samples
        avg_rssi = sum(rssi_samples) / num_rssi_samples

        # Prepare data packet and publish
        packet = {
            'rssi': avg_rssi
        }

        client.publish(client_topic, avg_rssi)
        time.sleep(rssi_delay)

if __name__ == '__main__':
    main()

