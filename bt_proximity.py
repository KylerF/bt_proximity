import bluetooth
import bluetooth._bluetooth as bt
import struct
import array
import fcntl


class BluetoothRSSI(object):
    """Object class for getting the RSSI value of a Bluetooth address.
    Reference: https://github.com/dagar/bluetooth-proximity
    """
    def __init__(self, addr):
        self.addr = addr
        self.hci_sock = bt.hci_open_dev()
        self.hci_fd = self.hci_sock.fileno()
        self.bt_sock = bluetooth.BluetoothSocket(bluetooth.L2CAP)
        self.bt_sock.settimeout(5)
        self.connected = False
        self.cmd_pkt = None

    def prep_cmd_pkt(self):
        """Prepares the command packet for requesting RSSI"""
        reqstr = struct.pack(
            "6sB17s", bt.str2ba(self.addr), bt.ACL_LINK, bytes("\0" * 17, "utf-8"))
        
        request = array.array("b", reqstr)
        handle = fcntl.ioctl(self.hci_fd, bt.HCIGETCONNINFO, request, 1)
        handle = struct.unpack("8xH14x", request.tostring())[0]
        self.cmd_pkt = struct.pack('H', handle)

    def connect(self):
        """Connects to the Bluetooth address"""
        self.bt_sock = bluetooth.BluetoothSocket(bluetooth.L2CAP)
        connection = self.bt_sock.connect_ex((self.addr, 1))  # PSM 1 - Service Discovery
        self.connected = (connection == 0 or connection == 106)

    def get_rssi(self):
        """Gets the current RSSI value.
        @return: The RSSI value (float) or None if the device connection fails
                 (i.e. the device is nowhere nearby).
        """
        try:
            self.connect()
            if not self.connected:
                return -99

            if self.cmd_pkt is None:
                self.prep_cmd_pkt()
            # Send command to request RSSI
            rssi = bt.hci_send_req(
                self.hci_sock, bt.OGF_STATUS_PARAM,
                bt.OCF_READ_RSSI, bt.EVT_CMD_COMPLETE, 4, self.cmd_pkt)
            
            rssi_int = rssi[3]
            rssi_bytes = rssi_int.to_bytes((rssi_int.bit_length() + 7) // 8, 'big')
            rssi = struct.unpack('b', rssi_bytes)[0]
            
            return rssi
        except IOError:
            # Happens if connection fails (e.g. device is not in range)
            self.connected = False
            
            return -99
        except struct.error:
            # Invalid RSSI value returned
            
            return None

