"""
Low-level command and state wrappers for Go2
"""

import struct
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import time


class LowCmdWrapper:
    """Wrapper for low-level command publishing"""

    def __init__(self):
        """Initialize low command wrapper"""
        self.msg_ = unitree_go_msg_dds__LowCmd_()
        self.crc = CRC()
        self._init_lowcmd()

        # Create publisher
        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()

        # Start publishing thread
        self.thread = RecurrentThread(
            interval=0.002,  # 500Hz
            target=self._publish,
            name="lowcmd_pub"
        )
        self.thread.Start()

    def _init_lowcmd(self):
        """Initialize low command message"""
        self.msg_.head[0] = 0xFE
        self.msg_.head[1] = 0xEF
        self.msg_.level_flag = 0xFF
        self.msg_.gpio = 0

        for i in range(20):
            self.msg_.motor_cmd[i].mode = 0x01  # PMSM mode
            self.msg_.motor_cmd[i].q = 0.0
            self.msg_.motor_cmd[i].kp = 0.0
            self.msg_.motor_cmd[i].dq = 0.0
            self.msg_.motor_cmd[i].kd = 0.0
            self.msg_.motor_cmd[i].tau = 0.0

    def _publish(self):
        """Publish low command"""
        self.msg_.crc = self.crc.Crc(self.msg_)
        self.publisher.Write(self.msg_)


class LowStateWrapper:
    """Wrapper for low-level state subscription"""

    def __init__(self):
        """Initialize low state wrapper"""
        self.msg_ = None
        self._joystick_wrapper = None

        # Create subscriber
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._handler, 10)

    def _handler(self, msg: LowState_):
        """Handle incoming low state message"""
        self.msg_ = msg
        if self._joystick_wrapper is None:
            self._joystick_wrapper = JoystickWrapper(msg)
        else:
            self._joystick_wrapper.update(msg)

    def wait_for_connection(self, timeout: float = 10.0):
        """Wait for connection to robot"""
        start_time = time.time()
        while self.msg_ is None:
            if time.time() - start_time > timeout:
                raise TimeoutError("Failed to connect to robot")
            time.sleep(0.1)

    @property
    def motor_state(self):
        """Get motor state"""
        return self.msg_.motor_state if self.msg_ else []

    @property
    def imu_state(self):
        """Get IMU state"""
        return self.msg_.imu_state if self.msg_ else None

    @property
    def joystick(self):
        """Get joystick wrapper"""
        return self._joystick_wrapper


class JoystickWrapper:
    """Wrapper for joystick data with convenient access methods"""

    def __init__(self, lowstate_msg: LowState_):
        """Initialize joystick wrapper"""
        self.msg = lowstate_msg
        self._parse_data()

    def update(self, lowstate_msg: LowState_):
        """Update joystick data"""
        self.msg = lowstate_msg
        self._parse_data()

    def _parse_data(self):
        """Parse wireless remote data"""
        if not self.msg or not self.msg.wireless_remote:
            self._lx = 0.0
            self._ly = 0.0
            self._rx = 0.0
            self._ry = 0.0
            self._btn_data1 = 0
            self._btn_data2 = 0
            return

        data = self.msg.wireless_remote

        # Parse stick values (float32 little-endian)
        try:
            self._lx = struct.unpack('<f', data[4:8])[0]
            self._rx = struct.unpack('<f', data[8:12])[0]
            self._ry = struct.unpack('<f', data[12:16])[0]
            self._ly = struct.unpack('<f', data[20:24])[0]
        except:
            self._lx = 0.0
            self._ly = 0.0
            self._rx = 0.0
            self._ry = 0.0

        # Parse button data
        try:
            self._btn_data1 = data[2]
            self._btn_data2 = data[3]
        except:
            self._btn_data1 = 0
            self._btn_data2 = 0

    def lx(self):
        """Left stick X axis"""
        return self._lx

    def ly(self):
        """Left stick Y axis"""
        return self._ly

    def rx(self):
        """Right stick X axis"""
        return self._rx

    def ry(self):
        """Right stick Y axis"""
        return self._ry

    @property
    def keys(self):
        """Button keys as bitmask (combined data1 and data2)"""
        return (self._btn_data1 << 8) | self._btn_data2

    @property
    def R1(self):
        """R1 button"""
        return (self._btn_data1 >> 0) & 1

    @property
    def L1(self):
        """L1 button"""
        return (self._btn_data1 >> 1) & 1

    @property
    def Start(self):
        """Start button"""
        return (self._btn_data1 >> 2) & 1

    @property
    def Select(self):
        """Select button"""
        return (self._btn_data1 >> 3) & 1

    @property
    def R2(self):
        """R2 button"""
        return (self._btn_data1 >> 4) & 1

    @property
    def L2(self):
        """L2 button"""
        return (self._btn_data1 >> 5) & 1

    @property
    def F1(self):
        """F1 button"""
        return (self._btn_data1 >> 6) & 1

    @property
    def F3(self):
        """F3 button"""
        return (self._btn_data1 >> 7) & 1

    @property
    def A(self):
        """A button"""
        return (self._btn_data2 >> 0) & 1

    @property
    def B(self):
        """B button"""
        return (self._btn_data2 >> 1) & 1

    @property
    def X(self):
        """X button"""
        return (self._btn_data2 >> 2) & 1

    @property
    def Y(self):
        """Y button"""
        return (self._btn_data2 >> 3) & 1

    @property
    def Up(self):
        """Up button"""
        return (self._btn_data2 >> 4) & 1

    @property
    def Right(self):
        """Right button"""
        return (self._btn_data2 >> 5) & 1

    @property
    def Down(self):
        """Down button"""
        return (self._btn_data2 >> 6) & 1

    @property
    def Left(self):
        """Left button"""
        return (self._btn_data2 >> 7) & 1
