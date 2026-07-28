#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

from rclpy import logging
from onrobot_rg_control.OnRobotRGControllerServer import OnRobotRGNode
import threading

try:
    # pymodbus >= 3
    from pymodbus.client import ModbusTcpClient
    DEVICE_ADDRESS_KEY = "slave"
except ImportError:
    # Ubuntu 22.04 / ROS 2 Humble commonly provides pymodbus 2.5.x.
    from pymodbus.client.sync import ModbusTcpClient
    DEVICE_ADDRESS_KEY = "unit"

from onrobot_rg_control.onrobot_rg_communication.comBase import OnRobotCommunicationBase

class communication(OnRobotCommunicationBase):
    """ communication sends commands and receives the status of RG gripper.

        Attributes:
            client (pymodbus.client.sync.ModbusTcpClient):
                instance of ModbusTcpClient to establish modbus connection

            connectToDevice: Connects to the client device (gripper).
            disconnectFromDevice: Closes connection.
            sendCommand: Sends a command to the Gripper.
            restartPowerCycle: Restarts the power cycle of Compute Box.
            getStatus: Sends a request to read and returns the gripper status.
    """

    def __init__(self, base : OnRobotRGNode):
        self.base = base
        self.client : ModbusTcpClient
        self.lock = threading.Lock()
        self.logger = logging.get_logger("OnRobot Modbus")

    def connectToDevice(self, ip, port : int, changer_addr=65):
        """ Connects to the client device (gripper).

            Args:
                ip (str): IP address (e.g. '192.168.1.1')
                port (str): port number (e.g. '502')
                changer_addr (int): quick tool changer address
        """
        
        self.changer_addr = changer_addr
        self.logger.info("Changer address: " + str(self.changer_addr))
        self.client = ModbusTcpClient(
            host=ip,
            port=port,
            timeout=1,
        )
        if not self.client.connect():
            message = f"Connection failed: {ip}:{port}"
            self.logger.error(message)
            raise ConnectionError(message)
        self.logger.info("Connected successfully.")

    def disconnectFromDevice(self):
        """ Closes connection. """
        
        self.client.close()

    def sendCommand(self, force: float, joint_angle: float, command_type: int):
        """ Sends a command to the Gripper.

            Args:
                joint angle (float): joint angle of target position
                force (int): force of movement
                command_type (int): command type based on movement
        """
        message: list[int] = [(int)(force*10), (int)(self.base.jointValueToWidth(joint_angle)*10000), command_type]
        self.logger.info((str)(message))
        self.logger.info("Send Command")
        # Sending a command to the device (address 0 ~ 2)
        with self.lock:
            result = self.client.write_registers(
                address=0,
                values=message,
                **{DEVICE_ADDRESS_KEY: self.changer_addr},
            )
        self._raise_on_error(result, "send command")

    def restartPowerCycle(self):
        """ Restarts the power cycle of Compute Box.

            Necessary is Safety Switch of the grippers are pressed
            Writing 2 to this field powers the tool off
            for a short amount of time and then powers them back
        """

        restart_address = 63

        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        with self.lock:
            result = self.client.write_register(
                address=0,
                value=2,
                **{DEVICE_ADDRESS_KEY: restart_address},
            )
        self._raise_on_error(result, "restart power cycle")

    def getStatus(self):
        """ Sends a request to read and returns the gripper status. """
        
        # Getting status from the device (address 258 ~ 275)
        with self.lock:
            result = self.client.read_holding_registers(
                address=258,
                count=18,
                **{DEVICE_ADDRESS_KEY: self.changer_addr},
            )
        self._raise_on_error(result, "read status")
        response = result.registers
        # Output the result

        return {
            'offset': ((float)(response[0]))/10,
            'depth' : ((float)(response[5]))/10,
            'relative_depth' : ((float)(response[6]))/10,
            'relative_width' : ((float)(response[9]))/10,
            'joint_angle' : self.base.widthToJointValue(((float)(response[9]))/10000),
            'busy' : response[10],
            'grip' : response[11],
            's1_p' : response[12],
            's1_t' : response[13],
            's2_p' : response[14],
            's2_t' : response[15],
            'safety' : response[16],           
            'width' : ((float)(response[17]))/10,
        }

    @staticmethod
    def _raise_on_error(result, operation: str):
        if result.isError():
            raise ConnectionError(f"Modbus {operation} failed: {result!r}")
