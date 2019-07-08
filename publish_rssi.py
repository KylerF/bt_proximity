"""
Bluetooth device proximity tracking.
Monitors the RSSI received from bluetooth devices and 
publishes to HomeAssistant's Mosquitto instance.
"""
from systemd.journal import JournalHandler
from bt_proximity import BluetoothRSSI
import paho.mqtt.client as mqtt
import bluetooth
import threading
import logging
import socket
import yaml
import json
import time
import sys
import os

# Configure logging to journal
log = logging.getLogger(__name__)

journald_handler = JournalHandler()
journald_handler.setFormatter(logging.Formatter(
    '[%(levelname)s] %(message)s'
))

log.addHandler(journald_handler)
log.setLevel(logging.DEBUG)

# MQTT connection callback
def on_connect(client, userdata, flags, rc):
    log.info("Connected to HASS: flags={0}, result_code={1}, client_id={2}".format(str(flags), str(rc), client_name))

# MQTT disconnected callback
def on_disconnect(client, userdata,rc=0):
    log.error("Disconnected from HASS with result code {}".format(str(rc)))

# Track and publish RSSI of a specific device
def publish_rssi(client, client_topic, mac, num_rssi_samples, rssi_delay):
    # Init RSSI samples array
    rssi_samples = [-99] * num_rssi_samples
    sample_num = 0

    while True:
        # Attempt connection to device and query RSSI
        btrssi = BluetoothRSSI(addr=mac)
        rssi = btrssi.get_rssi()

        # Default "away" value
        if not btrssi.connected:
            rssi = -99

        # Calculate the running mean
        rssi_samples[sample_num] = rssi
        sample_num = (sample_num + 1) % num_rssi_samples
        avg_rssi = sum(rssi_samples) / num_rssi_samples

        # Publish to HASS
        client.publish(client_topic, avg_rssi)
        time.sleep(rssi_delay)

def main():
    # Load configuration from file
    config_file = open('configuration.yaml')
    config = yaml.safe_load(config_file)
    mqtt_config = config['mqtt']
    scan_config = config['scan']
    rssi_config = config['rssi']

    # Load MQTT configuration
    broker = mqtt_config['url']
    port = mqtt_config['port']
    keepalive = mqtt_config['keepalive']
    client_id = mqtt_config['client_id']
    user = mqtt_config['user']
    pwd = mqtt_config['pwd']
    topic_prefix = mqtt_config['topic_prefix']

    # Load new device scanning configuration
    scan_duration = scan_config['duration']
    scan_delay = scan_config['delay']

    # Load RSSI configuration
    num_rssi_samples = rssi_config['samples']
    rssi_delay = rssi_config['delay']

    # Load current known devices
    devices_file = open('known_devices.yaml')
    known_devices = yaml.safe_load(devices_file)

    # Close file streams
    config_file.close()
    devices_file.close()

    # Connect to HomeAssistant
    client = mqtt.Client(client_id)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.username_pw_set(user, pwd)
    client.connect(broker, port, keepalive)
    client.loop_start()

    # Keep a list of tracked MAC addresses
    macs = []

    # Start tracking known devices, if there are any
    if not known_devices:
        known_devices = {}

    for device_id in known_devices: 
        device = known_devices[device_id]

        # Skip if tracking is disabled
        if not device['track']:
            continue
        
        client_topic = '{0}/{1}/{2}'.format(topic_prefix, client_id, device_id)
        mac = device['mac']
        args = (client, client_topic, mac, num_rssi_samples, rssi_delay)
        macs.append(mac)

        # Kick off a new thread
        tracking_thread = threading.Thread(target=publish_rssi, args=args)
        tracking_thread.start()

    # Periodically scan for new devices
    while True:
        discovered_devices = bluetooth.discover_devices(duration=scan_duration, lookup_names=True)

        for device in discovered_devices:
            mac = device[0]
            name = device[1]
            
            # Skip if we already know the scanned device
            if mac in macs:
                continue

            # Add the new device
            log.info('Adding new device: {0} {1}'.format(mac, name))
            device_id = name.lower().replace("'", "").replace(" ", "_")
            new_device = {
                device_id: {
                    'mac': mac, 
                    'name': name, 
                    'track': True
                }
            }
            known_devices.update(new_device)
            devices_file = open('known_devices.yaml', 'w')
            yaml.dump(known_devices, devices_file, default_flow_style=False)
            devices_file.close()

            client_topic = '{0}/{1}/{2}'.format(topic_prefix, client_id, device_id)
            
            args = (client, client_topic, mac, num_rssi_samples, rssi_delay)
            macs.append(mac)

            # Kick off a new thread
            tracking_thread = threading.Thread(target=publish_rssi, args=args)
            tracking_thread.start()
        
        time.sleep(scan_delay)

if __name__ == '__main__':
    main()

