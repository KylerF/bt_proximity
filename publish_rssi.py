"""
Bluetooth device proximity tracking.
Monitors the RSSI received from bluetooth devices and 
publishes to HomeAssistant's Mosquitto instance.
"""
from systemd.journal import JournalHandler
from bt_proximity import BluetoothRSSI
import paho.mqtt.client as mqtt
import threading
import logging
import socket
import json
import time
import sys
import os

# TODO Move to a database!!!
BT_ADDR = 'C4:98:80:35:B6:1D'

# Configure logging to journal
log = logging.getLogger(__name__)

journald_handler = JournalHandler()
journald_handler.setFormatter(logging.Formatter(
    '[%(levelname)s] %(message)s'
))

log.addHandler(journald_handler)
log.setLevel(logging.DEBUG)

# MQTT client connection details
broker = '192.168.1.5'
client_name = socket.gethostname()
client = mqtt.Client(client_name)
client.username_pw_set("kyler", "*()IOP890iop")

# MQTT publish details
client_topic = "pressence/{}/kyler".format(client_name)
publish_interval = 1

# RSSI averaging params
num_rssi_samples = 10  # Number of samples collected
rssi_delay       = 0.5 # Delay between measurements (sec)

# MQTT connection callback
def on_connect(client, userdata, flags, rc):
    log.info("Connected to HASS: flags={0}, result_code={1}, client_id={2}".format(str(flags), str(rc), client_name))

# MQTT data published callback
def on_disconnect(client, userdata,rc=0):
    log.error("Disconnected from HASS with result code {}".format(str(rc)))

client.on_connect = on_connect
client.on_disconnect = on_disconnect

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
        rssi = btrssi.get_rssi()

        # Default "away" value
        if not btrssi.connected:
            rssi = -99

        # Calculate the running mean
        rssi_samples[sample_num] = rssi
        sample_num = (sample_num + 1) % num_rssi_samples
        avg_rssi = sum(rssi_samples) / num_rssi_samples

        client.publish(client_topic, avg_rssi)
        time.sleep(rssi_delay)

if __name__ == '__main__':
    main()

