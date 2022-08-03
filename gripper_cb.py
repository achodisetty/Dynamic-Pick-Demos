"""Module to control Robotiq's grippers with the I/O coupler for CB series- tested with HAND-CB"""

import rtde_io
# import rtde_receive
import time


class RobotiqGripper_CB:
    """
    Communicates with the Robotiq Adaptive gripper(Conencted with the I/O coupler for CB series)
    """

    DIGITAL_TOOL_OUTPUT_0 = 0
    DIGITAL_TOOL_OUTPUT_1 = 1

    def __init__(self) -> None:
        self.ip = "192.168.1.199"

    def connect (self, hostname = "192.168.1.199") -> None:
        """
        Connectes to the gripper with the given IP

        Args:
            hostname (str, optional): Hostname or ip.. Defaults to "192.168.1.199".
        """
        self.ip = hostname
        self.rtde_io_ = rtde_io.RTDEIOInterface(self.ip)

    def activate (self, activation_needed = True) -> None:
        """
        Activates the gripper after a power cycle

        Args:
            activation_needed (bool, optional): If False then the gripper is already active. Defaults to True.
        """
        if activation_needed:
            self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_0, False)
            self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_1, False)
            time.sleep(0.05)

            self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_0, True)
            time.sleep(0.05)

            self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_0, False)
            time.sleep(0.05)

            self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_1, True)
            time.sleep(0.05)

            self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_1, False)
            time.sleep(0.05)

    def open (self) -> None:
        """
        Opens the gripper
        """
        self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_1, False)
        self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_0, True)

    def close (self) -> None:
        """
        Closes the gripper
        """
        self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_1, False)
        self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_0, False)

    def open_slow (self) -> None:
        """
        Opens the gripper slowly
        """
        self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_1, True)
        self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_0, True)

    def close_slow (self) -> None:
        """
        Closes the gripper slowly
        """
        self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_1, True)
        self.rtde_io_.setToolDigitalOut(self.DIGITAL_TOOL_OUTPUT_0, False)

    def open_and_wait (self, wait_time = 0.01) -> None:
        """
        Opens the gripper and waits for the said time

        Args:
            wait_time (float, optional): Wait time for the gripper to open. Defaults to 0.01.
        """
        self._open()
        time.sleep(wait_time)

    def close_and_wait (self, wait_time = 0.01) -> None:
        """
        Closes the gripper and waits for the said time

        Args:
            wait_time (float, optional): Wait time for the gripper to close. Defaults to 0.01.
        """
        self._close()
        time.sleep(wait_time)