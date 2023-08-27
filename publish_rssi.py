"""
Bluetooth device proximity tracking.
Monitors the RSSI received from bluetooth devices and 
publishes to HomeAssistant's Mosquitto instance.
"""

from bt_proximity import BluetoothRSSI
import paho.mqtt.client as mqtt
import bluetooth
import threading
import traceback
import logging
import yaml
import time
import sys
import os

# Configure logging to journal
log = logging.getLogger(__name__)

log_handler = logging.StreamHandler()
log_handler.setFormatter(logging.Formatter(
    '[%(levelname)s] %(message)s'
))

log.addHandler(log_handler)

"""
Config file loader/validator
"""
class ConfigLoader:
    def __init__(self, filename):
        self.filename = filename

    # Get all config options
    def load(self):
        # Load configuration from file
        try:
            self.config_file = open(self.filename)
        except:
            log.error('Unable to open config file {}'.format(self.filename))
            self.handle_fatal_error()

        try:
            config = yaml.safe_load(self.config_file)
        except:
            log.error('Unable to parse config file {} - check your syntax'.format(self.filename))
            self.handle_fatal_error()

        try:
            log_config = config['logger']
            mqtt_config = config['mqtt']
            scan_config = config['scan']
            rssi_config = config['rssi']

            # Load logger configuration
            self.loglevel = log_config['level']

            # Load MQTT configuration
            self.broker = mqtt_config['url']
            self.port = mqtt_config['port']
            self.keepalive = mqtt_config['keepalive']
            self.client_id = mqtt_config['client_id']
            self.user = mqtt_config['user']
            self.pwd = mqtt_config['pwd']
            self.topic_prefix = mqtt_config['topic_prefix']

            # Load new device scanning configuration
            self.scan_enabled = scan_config['enabled']
            self.scan_duration = scan_config['duration']
            self.scan_delay = scan_config['delay']

            # Load RSSI configuration
            self.use_mean = rssi_config['use_mean']
            self.num_rssi_samples = rssi_config['samples']
            self.max_delta = rssi_config['max_delta']
            self.rssi_delay = rssi_config['delay'] 
        except KeyError:
            log.error('Your configuration is incomplete!')
            self.handle_fatal_error()

        # Close the file stream
        self.config_file.close()

    # Catch for config errors. Prints trace and exits.
    def handle_fatal_error(self):
        traceback.print_exc()
        log.error('Configuration error detected. Shutting down.')
        sys.exit()

"""
Wrapper for bluetooth device information with handlers
for querying RSSI
"""
class Device:
    def __init__(self, device_id, mac, name, track=False):
        self.id = device_id
        self.mac = mac
        self.name = name
        self.track = track
        self.btrssi = BluetoothRSSI(addr=mac)

    # Query received bluetooth power (RSSI)
    def get_rssi(self):
        # Attempt connection to device and query RSSI
        rssi = self.btrssi.get_rssi()
        
        return rssi

"""
Holds all known devices and handles registration
of new devices
"""
class DeviceRegistry:
    def __init__(self):
        # Dict of all known devices, indexed by MAC
        self.known_devices = {}

        # Registry file name
        self.filename = 'known_devices.yaml'
    
    # Read all known devices from a YAML file and return a new
    # DeviceRegistry object
    @classmethod
    def load(self):
        registry = DeviceRegistry()

        # Create known_devices file if needed
        if not os.path.exists(registry.filename):
            log.info('No device registry found. Creating one in {}'.format(registry.filename))
            open(registry.filename, 'w')
            
            return registry
        else:
            try:
                registry.known_devices = {}
                devices_file = open(registry.filename, 'r')
            
                # Register all devices in the file
                known_devices_dict = yaml.safe_load(devices_file)
                
                # Return if empty
                if not known_devices_dict:
                    return registry

                for device_id in known_devices_dict:
                    device_dict = known_devices_dict[device_id]
                    mac = device_dict['mac']
                    name = device_dict['name']
                    track = device_dict['track']

                    registry.known_devices[mac] = Device(device_id, mac, name, track)
            except:
                log.error('Unable to read device registry file {}'.format(registry.filename))
                traceback.print_exc()
            finally:
                # Close file stream
                devices_file.close()

        # Allow instantiation by calling this method
        return registry

    # Register a new device
    def register(self, device):
        if not device.mac in self.known_devices:
            self.known_devices[device.mac] = device
            
            # Update the file with the new device
            self.update()

    # Remove a device from the registry
    def remove(self, device):
        try:
            del self.known_devices[device.mac]
            self.update()
        except KeyError:
            log.warn('Attemped to remove a device that is no longer registered! MAC: {}'.format(device.mac))

    # Stop tracking a device, but keep in in registry
    def stop_tracking(self, device):
        try:
            self.known_devices[device.mac].track = False
            self.update()
        except KeyError:
            log.warn('Attempted to stop tracking a device that is no longer registered! MAC: {}'.format(device.mac))

    # Update registry file with current devices
    def update(self):
        yaml_snapshot = self.to_yaml()

        try:
            devices_file = open(self.filename, 'w')
            yaml.dump(yaml_snapshot, devices_file, default_flow_style=False)
        except:
            log.error('Failed writing to device registry file {}'.format(self.filename))
            traceback.print_exc()
        finally:
            devices_file.close()

    # Convert the entire registry to a simple dict
    def to_yaml(self):
        registry_yaml = {}

        for mac in self.known_devices:
            device = self.known_devices[mac]
            device_yaml = {
                device.id: {
                    'mac': device.mac, 
                    'name': device.name, 
                    'track': device.track
                }
            }
            registry_yaml.update(device_yaml)

        return registry_yaml
"""
Wrapper class for MQTT client connection
"""
class HASSClient:
    # Create a new client object
    def __init__(self, client_id):
        self.client_id = client_id
        self.client = mqtt.Client(client_id)
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

    # Connect to server
    def connect(self, url, port, keepalive, user, pwd):
        self.client.username_pw_set(user, pwd)
        self.client.connect(url, port, keepalive)
        self.client.loop_start()

    # Publish a message
    def publish(self, topic, message):
        self.client.publish(topic, message)

    # Callback functions for MQTT connection
    def on_connect(self, client, userdata, flags, rc):
        log.info('Connected to HASS: flags={0}, result_code={1}, client_id={2}'.format(str(flags), str(rc), self.client_id))

    def on_publish(self, client, userdata, mid):
        log.debug('Data published to HASS: mid={}'.format(str(mid)))

    def on_message(self, client, userdata, message):
        log.debug('Message received from HASS: message={}'.format(message.payload))

    def on_disconnect(client, userdata,rc=0):
        log.error("Disconnected from HASS with result code {}".format(str(rc)))

"""
Threaded execution for tracking a single device
and publishing results to HASS
"""
class TrackingThread:
    def __init__(self, device, config, hass_client):
        self.config = config
        self.device = device
        self.hass_client = hass_client

        # Add an event to allow this thread to be stopped
        self.stop_event = threading.Event()
        
        # Extract MQTT details from config
        self.mqtt_client_id = config.client_id
        self.mqtt_topic_prefix = config.topic_prefix

        # Extract RSSI averaging params from config
        self.use_mean = config.use_mean
        self.num_rssi_samples = config.num_rssi_samples
        self.max_delta = config.max_delta
        self.rssi_delay = config.rssi_delay

        log.debug("Tracking device {0} - client_id: {1}, num_rssi_samples: {2}, rssi_delay: {3}".format(
            self.device.name, self.mqtt_client_id, self.num_rssi_samples, self.rssi_delay        
        ))

    # Keep a running average RSSI from the device and publish to HASS
    def publish_rssi(self):
        # Init RSSI samples array
        rssi_samples = [-99] * self.num_rssi_samples
        sample_num = 0

        # Loop until a 'stop' event is received
        while not self.stop_event.is_set():
            # Attempt connection to device and query RSSI
            rssi = self.device.get_rssi()
            
            # Ignore bad readings
            if rssi == None:
                log.debug("Invalid RSSI value returned!")
                time.sleep(self.rssi_delay)
                continue 

            log.debug("{0} RSSI: {1}".format(self.device.name, rssi))

            # Calculate the running mean
            rssi_samples[sample_num] = rssi
            sample_num = (sample_num + 1) % self.num_rssi_samples
            avg_rssi = sum(rssi_samples) / self.num_rssi_samples
            log.debug("{0} Average: {1}".format(self.device.name, avg_rssi))
            
            if self.use_mean: 
                # Calculate the running mean
                rssi_samples[sample_num] = rssi
                sample_num = (sample_num + 1) % self.num_rssi_samples
                rssi = sum(rssi_samples) / self.num_rssi_samples

                # Ignore readings with large delta
                last_reading = rssi_samples[sample_num-1]
                delta = abs(rssi - last_reading)

                if abs(delta > self.max_delta):
                    log.debug("Delta too large. Skipping.")
                    time.sleep(self.rssi_delay)
                    continue

            log.debug('{0} RSSI: {1}'.format(self.device.name, rssi))
            
            # Publish to HASS
            self.hass_client.publish(self.mqtt_client_topic, rssi)
            time.sleep(self.rssi_delay)

    # Start tracking the device
    def start(self):
        # Create a client connection to HASS
        self.mqtt_client_topic = '{0}/{1}/{2}'.format(self.mqtt_topic_prefix, self.mqtt_client_id, self.device.id)

        # Start the new thread
        self.thread_obj = threading.Thread(target=self.publish_rssi)
        self.thread_obj.start()

    # Stop tracking the device
    def stop(self):
        self.stop_event.set()

"""
Threaded execution for scanning for new devices
and adding them to the registry
"""
class ScanningThread:
    def __init__(self, device_registry, scan_duration, scan_delay):
        self.device_registry = device_registry
        self.scan_duration = scan_duration
        self.scan_delay = scan_delay

    # Looping target function for thread
    def loop(self):
        macs = []

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

    # Start periodic scanning
    def start(self):
        pass

    # Stop periodic scanning
    def stop(self):
        pass

def main():
    # Load configuration from file
    config = ConfigLoader('configuration.yaml')
    config.load()

    # Set log level from config
    log.setLevel(config.loglevel)

    # Load MQTT configuration
    broker = config.broker
    port = config.port
    keepalive = config.keepalive
    client_id = config.client_id
    user = config.user
    pwd = config.pwd
    topic_prefix = config.topic_prefix

    # Load new device scanning configuration
    scan_enabled = config.scan_enabled
    scan_duration = config.scan_duration
    scan_delay = config.scan_delay

    # Load RSSI configuration
    num_rssi_samples = config.num_rssi_samples
    rssi_delay = config.rssi_delay

    # Load current known devices
    device_registry = DeviceRegistry.load()
    known_devices = device_registry.known_devices

    # Create shared client connection to HASS
    hass_client = HASSClient(client_id)
    hass_client.connect(broker, port, keepalive, user, pwd)

    # Keep a list of tracked MAC addresses
    macs = []

    # Start tracking known devices, if there are any
    for device_id in known_devices:
        device = known_devices[device_id]
        
        if not device.track:
            continue

        # Kick off a new thread
        tracking_thread = TrackingThread(device, config, hass_client)
        tracking_thread.start()
    
    # Periodically scan for new devices
    while True:
        discovered_devices = []

        if scan_enabled:
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
            
            new_device = Device(device_id, mac, name, track=False)
            device_registry.register(new_device)

            #client_topic = '{0}/{1}/{2}'.format(topic_prefix, client_id, device_id)
 
            #args = (client, client_topic, mac, num_rssi_samples, rssi_delay)
            #macs.append(mac)

            # Kick off a new thread
            #tracking_thread = threading.Thread(target=publish_rssi, args=args)
            #tracking_thread.start()

        time.sleep(scan_delay)

if __name__ == '__main__':
    main()

