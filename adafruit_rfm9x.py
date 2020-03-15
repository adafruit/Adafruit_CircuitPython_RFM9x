# The MIT License (MIT)
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_rfm9x`
====================================================

CircuitPython module for the RFM95/6/7/8 LoRa 433/915mhz radio modules.  This is
adapted from the Radiohead library RF95 code from:
http: www.airspayce.com/mikem/arduino/RadioHead/

* Author(s): Tony DiCola, Jerry Needell
"""
import time
import digitalio
from micropython import const

try:
    from warnings import warn
except ImportError:

    def warn(msg, **kwargs):
        "Issue a warning to stdout."
        print(
            "%s: %s"
            % ("Warning" if kwargs.get("cat") is None else kwargs["cat"].__name__, msg)
        )


import adafruit_bus_device.spi_device as spidev


__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_RFM9x.git"


# pylint: disable=bad-whitespace
# Internal constants:
# Register names (FSK Mode even though we use LoRa instead, from table 85)
_RH_RF95_REG_00_FIFO = const(0x00)
_RH_RF95_REG_01_OP_MODE = const(0x01)
_RH_RF95_REG_02_RESERVED = const(0x02)
_RH_RF95_REG_03_RESERVED = const(0x03)
_RH_RF95_REG_04_RESERVED = const(0x04)
_RH_RF95_REG_05_RESERVED = const(0x05)
_RH_RF95_REG_06_FRF_MSB = const(0x06)
_RH_RF95_REG_07_FRF_MID = const(0x07)
_RH_RF95_REG_08_FRF_LSB = const(0x08)
_RH_RF95_REG_09_PA_CONFIG = const(0x09)
_RH_RF95_REG_0A_PA_RAMP = const(0x0A)
_RH_RF95_REG_0B_OCP = const(0x0B)
_RH_RF95_REG_0C_LNA = const(0x0C)
_RH_RF95_REG_0D_FIFO_ADDR_PTR = const(0x0D)
_RH_RF95_REG_0E_FIFO_TX_BASE_ADDR = const(0x0E)
_RH_RF95_REG_0F_FIFO_RX_BASE_ADDR = const(0x0F)
_RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR = const(0x10)
_RH_RF95_REG_11_IRQ_FLAGS_MASK = const(0x11)
_RH_RF95_REG_12_IRQ_FLAGS = const(0x12)
_RH_RF95_REG_13_RX_NB_BYTES = const(0x13)
_RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB = const(0x14)
_RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB = const(0x15)
_RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB = const(0x16)
_RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB = const(0x17)
_RH_RF95_REG_18_MODEM_STAT = const(0x18)
_RH_RF95_REG_19_PKT_SNR_VALUE = const(0x19)
_RH_RF95_REG_1A_PKT_RSSI_VALUE = const(0x1A)
_RH_RF95_REG_1B_RSSI_VALUE = const(0x1B)
_RH_RF95_REG_1C_HOP_CHANNEL = const(0x1C)
_RH_RF95_REG_1D_MODEM_CONFIG1 = const(0x1D)
_RH_RF95_REG_1E_MODEM_CONFIG2 = const(0x1E)
_RH_RF95_REG_1F_SYMB_TIMEOUT_LSB = const(0x1F)
_RH_RF95_REG_20_PREAMBLE_MSB = const(0x20)
_RH_RF95_REG_21_PREAMBLE_LSB = const(0x21)
_RH_RF95_REG_22_PAYLOAD_LENGTH = const(0x22)
_RH_RF95_REG_23_MAX_PAYLOAD_LENGTH = const(0x23)
_RH_RF95_REG_24_HOP_PERIOD = const(0x24)
_RH_RF95_REG_25_FIFO_RX_BYTE_ADDR = const(0x25)
_RH_RF95_REG_26_MODEM_CONFIG3 = const(0x26)

_RH_RF95_REG_40_DIO_MAPPING1 = const(0x40)
_RH_RF95_REG_41_DIO_MAPPING2 = const(0x41)
_RH_RF95_REG_42_VERSION = const(0x42)

_RH_RF95_REG_4B_TCXO = const(0x4B)
_RH_RF95_REG_4D_PA_DAC = const(0x4D)
_RH_RF95_REG_5B_FORMER_TEMP = const(0x5B)
_RH_RF95_REG_61_AGC_REF = const(0x61)
_RH_RF95_REG_62_AGC_THRESH1 = const(0x62)
_RH_RF95_REG_63_AGC_THRESH2 = const(0x63)
_RH_RF95_REG_64_AGC_THRESH3 = const(0x64)

# RH_RF95_REG_01_OP_MODE                             0x01
_RH_RF95_LONG_RANGE_MODE = const(0x80)
_RH_RF95_ACCESS_SHARED_REG = const(0x40)
_RH_RF95_MODE = const(0x07)
_RH_RF95_MODE_SLEEP = const(0x00)
_RH_RF95_MODE_STDBY = const(0x01)
_RH_RF95_MODE_FSTX = const(0x02)
_RH_RF95_MODE_TX = const(0x03)
_RH_RF95_MODE_FSRX = const(0x04)
_RH_RF95_MODE_RXCONTINUOUS = const(0x05)
_RH_RF95_MODE_RXSINGLE = const(0x06)
_RH_RF95_MODE_CAD = const(0x07)

# RH_RF95_REG_09_PA_CONFIG                           0x09
_RH_RF95_PA_SELECT = const(0x80)
_RH_RF95_MAX_POWER = const(0x70)
_RH_RF95_OUTPUT_POWER = const(0x0F)

# RH_RF95_REG_0A_PA_RAMP                             0x0a
_RH_RF95_LOW_PN_TX_PLL_OFF = const(0x10)
_RH_RF95_PA_RAMP = const(0x0F)
_RH_RF95_PA_RAMP_3_4MS = const(0x00)
_RH_RF95_PA_RAMP_2MS = const(0x01)
_RH_RF95_PA_RAMP_1MS = const(0x02)
_RH_RF95_PA_RAMP_500US = const(0x03)
_RH_RF95_PA_RAMP_250US = const(0x04)
_RH_RF95_PA_RAMP_125US = const(0x05)
_RH_RF95_PA_RAMP_100US = const(0x06)
_RH_RF95_PA_RAMP_62US = const(0x07)
_RH_RF95_PA_RAMP_50US = const(0x08)
_RH_RF95_PA_RAMP_40US = const(0x09)
_RH_RF95_PA_RAMP_31US = const(0x0A)
_RH_RF95_PA_RAMP_25US = const(0x0B)
_RH_RF95_PA_RAMP_20US = const(0x0C)
_RH_RF95_PA_RAMP_15US = const(0x0D)
_RH_RF95_PA_RAMP_12US = const(0x0E)
_RH_RF95_PA_RAMP_10US = const(0x0F)

# RH_RF95_REG_0B_OCP                                 0x0b
_RH_RF95_OCP_ON = const(0x20)
_RH_RF95_OCP_TRIM = const(0x1F)

# RH_RF95_REG_0C_LNA                                 0x0c
_RH_RF95_LNA_GAIN = const(0xE0)
_RH_RF95_LNA_BOOST = const(0x03)
_RH_RF95_LNA_BOOST_DEFAULT = const(0x00)
_RH_RF95_LNA_BOOST_150PC = const(0x11)

# RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
_RH_RF95_RX_TIMEOUT_MASK = const(0x80)
_RH_RF95_RX_DONE_MASK = const(0x40)
_RH_RF95_PAYLOAD_CRC_ERROR_MASK = const(0x20)
_RH_RF95_VALID_HEADER_MASK = const(0x10)
_RH_RF95_TX_DONE_MASK = const(0x08)
_RH_RF95_CAD_DONE_MASK = const(0x04)
_RH_RF95_FHSS_CHANGE_CHANNEL_MASK = const(0x02)
_RH_RF95_CAD_DETECTED_MASK = const(0x01)

# RH_RF95_REG_12_IRQ_FLAGS                           0x12
_RH_RF95_RX_TIMEOUT = const(0x80)
_RH_RF95_RX_DONE = const(0x40)
_RH_RF95_PAYLOAD_CRC_ERROR = const(0x20)
_RH_RF95_VALID_HEADER = const(0x10)
_RH_RF95_TX_DONE = const(0x08)
_RH_RF95_CAD_DONE = const(0x04)
_RH_RF95_FHSS_CHANGE_CHANNEL = const(0x02)
_RH_RF95_CAD_DETECTED = const(0x01)

# RH_RF95_REG_18_MODEM_STAT                          0x18
_RH_RF95_RX_CODING_RATE = const(0xE0)
_RH_RF95_MODEM_STATUS_CLEAR = const(0x10)
_RH_RF95_MODEM_STATUS_HEADER_INFO_VALID = const(0x08)
_RH_RF95_MODEM_STATUS_RX_ONGOING = const(0x04)
_RH_RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED = const(0x02)
_RH_RF95_MODEM_STATUS_SIGNAL_DETECTED = const(0x01)

# RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
_RH_RF95_PLL_TIMEOUT = const(0x80)
_RH_RF95_RX_PAYLOAD_CRC_IS_ON = const(0x40)
_RH_RF95_FHSS_PRESENT_CHANNEL = const(0x3F)

# RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
_RH_RF95_BW = const(0xC0)
_RH_RF95_BW_125KHZ = const(0x00)
_RH_RF95_BW_250KHZ = const(0x40)
_RH_RF95_BW_500KHZ = const(0x80)
_RH_RF95_BW_RESERVED = const(0xC0)
_RH_RF95_CODING_RATE = const(0x38)
_RH_RF95_CODING_RATE_4_5 = const(0x00)
_RH_RF95_CODING_RATE_4_6 = const(0x08)
_RH_RF95_CODING_RATE_4_7 = const(0x10)
_RH_RF95_CODING_RATE_4_8 = const(0x18)
_RH_RF95_IMPLICIT_HEADER_MODE_ON = const(0x04)
_RH_RF95_RX_PAYLOAD_CRC_ON = const(0x02)
_RH_RF95_LOW_DATA_RATE_OPTIMIZE = const(0x01)

# RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
_RH_RF95_DETECTION_OPTIMIZE = const(0x31)
_RH_RF95_DETECTION_THRESHOLD = const(0x37)
_RH_RF95_SPREADING_FACTOR = const(0xF0)
_RH_RF95_SPREADING_FACTOR_64CPS = const(0x60)
_RH_RF95_SPREADING_FACTOR_128CPS = const(0x70)
_RH_RF95_SPREADING_FACTOR_256CPS = const(0x80)
_RH_RF95_SPREADING_FACTOR_512CPS = const(0x90)
_RH_RF95_SPREADING_FACTOR_1024CPS = const(0xA0)
_RH_RF95_SPREADING_FACTOR_2048CPS = const(0xB0)
_RH_RF95_SPREADING_FACTOR_4096CPS = const(0xC0)
_RH_RF95_TX_CONTINUOUS_MOE = const(0x08)
_RH_RF95_AGC_AUTO_ON = const(0x04)
_RH_RF95_SYM_TIMEOUT_MSB = const(0x03)

# RH_RF95_REG_4D_PA_DAC                              0x4d
_RH_RF95_PA_DAC_DISABLE = const(0x04)
_RH_RF95_PA_DAC_ENABLE = const(0x07)

# The crystal oscillator frequency of the module
_RH_RF95_FXOSC = 32000000.0

# The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
_RH_RF95_FSTEP = _RH_RF95_FXOSC / 524288

# RadioHead specific compatibility constants.
_RH_BROADCAST_ADDRESS = const(0xFF)

# User facing constants:
SLEEP_MODE = 0b000
STANDBY_MODE = 0b001
FS_TX_MODE = 0b010
TX_MODE = 0b011
FS_RX_MODE = 0b100
RX_MODE = 0b101
# pylint: enable=bad-whitespace


# Disable the too many instance members warning.  Pylint has no knowledge
# of the context and is merely guessing at the proper amount of members.  This
# is a complex chip which requires exposing many attributes and state.  Disable
# the warning to work around the error.
# pylint: disable=too-many-instance-attributes


class RFM9x:
    """Interface to a RFM95/6/7/8 LoRa radio module.  Allows sending and
    receivng bytes of data in long range LoRa mode at a support board frequency
    (433/915mhz).

    You must specify the following parameters:
    - spi: The SPI bus connected to the radio.
    - cs: The CS pin DigitalInOut connected to the radio.
    - reset: The reset/RST pin DigialInOut connected to the radio.
    - frequency: The frequency (in mhz) of the radio module (433/915mhz typically).

    You can optionally specify:
    - preamble_length: The length in bytes of the packet preamble (default 8).
    - high_power: Boolean to indicate a high power board (RFM95, etc.).  Default
    is True for high power.
    - baudrate: Baud rate of the SPI connection, default is 10mhz but you might
    choose to lower to 1mhz if using long wires or a breadboard.

    Remember this library makes a best effort at receiving packets with pure
    Python code.  Trying to receive packets too quickly will result in lost data
    so limit yourself to simple scenarios of sending and receiving single
    packets at a time.

    Also note this library tries to be compatible with raw RadioHead Arduino
    library communication. This means the library sets up the radio modulation
    to match RadioHead's defaults. Features like addressing and guaranteed
    delivery need to be implemented at an application level.
    """

    # Global buffer to hold data sent and received with the chip.  This must be
    # at least as large as the FIFO on the chip (256 bytes)!  Keep this on the
    # class level to ensure only one copy ever exists (with the trade-off that
    # this is NOT re-entrant or thread safe code by design).
    _BUFFER = bytearray(10)

    class _RegisterBits:
        # Class to simplify access to the many configuration bits avaialable
        # on the chip's registers.  This is a subclass here instead of using
        # a higher level module to increase the efficiency of memory usage
        # (all of the instances of this bit class will share the same buffer
        # used by the parent RFM69 class instance vs. each having their own
        # buffer and taking too much memory).

        # Quirk of pylint that it requires public methods for a class.  This
        # is a decorator class in Python and by design it has no public methods.
        # Instead it uses dunder accessors like get and set below.  For some
        # reason pylint can't figure this out so disable the check.
        # pylint: disable=too-few-public-methods

        # Again pylint fails to see the true intent of this code and warns
        # against private access by calling the write and read functions below.
        # This is by design as this is an internally used class.  Disable the
        # check from pylint.
        # pylint: disable=protected-access

        def __init__(self, address, *, offset=0, bits=1):
            assert 0 <= offset <= 7
            assert 1 <= bits <= 8
            assert (offset + bits) <= 8
            self._address = address
            self._mask = 0
            for _ in range(bits):
                self._mask <<= 1
                self._mask |= 1
            self._mask <<= offset
            self._offset = offset

        def __get__(self, obj, objtype):
            reg_value = obj._read_u8(self._address)
            return (reg_value & self._mask) >> self._offset

        def __set__(self, obj, val):
            reg_value = obj._read_u8(self._address)
            reg_value &= ~self._mask
            reg_value |= (val & 0xFF) << self._offset
            obj._write_u8(self._address, reg_value)

    operation_mode = _RegisterBits(_RH_RF95_REG_01_OP_MODE, bits=3)

    low_frequency_mode = _RegisterBits(_RH_RF95_REG_01_OP_MODE, offset=3, bits=1)

    modulation_type = _RegisterBits(_RH_RF95_REG_01_OP_MODE, offset=5, bits=2)

    # Long range/LoRa mode can only be set in sleep mode!
    long_range_mode = _RegisterBits(_RH_RF95_REG_01_OP_MODE, offset=7, bits=1)

    output_power = _RegisterBits(_RH_RF95_REG_09_PA_CONFIG, bits=4)

    max_power = _RegisterBits(_RH_RF95_REG_09_PA_CONFIG, offset=4, bits=3)

    pa_select = _RegisterBits(_RH_RF95_REG_09_PA_CONFIG, offset=7, bits=1)

    pa_dac = _RegisterBits(_RH_RF95_REG_4D_PA_DAC, bits=3)

    dio0_mapping = _RegisterBits(_RH_RF95_REG_40_DIO_MAPPING1, offset=6, bits=2)

    tx_done = _RegisterBits(_RH_RF95_REG_12_IRQ_FLAGS, offset=3, bits=1)

    rx_done = _RegisterBits(_RH_RF95_REG_12_IRQ_FLAGS, offset=6, bits=1)

    crc_error = _RegisterBits(_RH_RF95_REG_12_IRQ_FLAGS, offset=5, bits=1)

    bw_bins = (7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000)

    def __init__(
        self,
        spi,
        cs,
        reset,
        frequency,
        *,
        preamble_length=8,
        high_power=True,
        baudrate=5000000
    ):
        self.high_power = high_power
        # Device support SPI mode 0 (polarity & phase = 0) up to a max of 10mhz.
        # Set Default Baudrate to 5MHz to avoid problems
        self._device = spidev.SPIDevice(spi, cs, baudrate=baudrate, polarity=0, phase=0)
        # Setup reset as a digital input (default state for reset line according
        # to the datasheet).  This line is pulled low as an output quickly to
        # trigger a reset.  Note that reset MUST be done like this and set as
        # a high impedence input or else the chip cannot change modes (trust me!).
        self._reset = reset
        self._reset.switch_to_input(pull=digitalio.Pull.UP)
        self.reset()
        # No device type check!  Catch an error from the very first request and
        # throw a nicer message to indicate possible wiring problems.
        version = self._read_u8(_RH_RF95_REG_42_VERSION)
        if version != 18:
            raise RuntimeError(
                "Failed to find rfm9x with expected version -- check wiring"
            )

        # Set sleep mode, wait 10s and confirm in sleep mode (basic device check).
        # Also set long range mode (LoRa mode) as it can only be done in sleep.
        self.sleep()
        time.sleep(0.01)
        self.long_range_mode = True
        if self.operation_mode != SLEEP_MODE or not self.long_range_mode:
            raise RuntimeError("Failed to configure radio for LoRa mode, check wiring!")
        # clear default setting for access to LF registers if frequency > 525MHz
        if frequency > 525:
            self.low_frequency_mode = 0
        # Setup entire 256 byte FIFO
        self._write_u8(_RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0x00)
        self._write_u8(_RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0x00)
        # Set mode idle
        self.idle()
        # Defaults set modem config to RadioHead compatible Bw125Cr45Sf128 mode.
        self.signal_bandwidth = 125000
        self.coding_rate = 5
        self.spreading_factor = 7
        # Default to disable CRC checking on incoming packets.
        self.enable_crc = False
        # Note no sync word is set for LoRa mode either!
        self._write_u8(_RH_RF95_REG_26_MODEM_CONFIG3, 0x00)  # Preamble lsb?
        # Set preamble length (default 8 bytes to match radiohead).
        self.preamble_length = preamble_length
        # Set frequency
        self.frequency_mhz = frequency
        # Set TX power to low defaut, 13 dB.
        self.tx_power = 13

    # pylint: disable=no-member
    # Reconsider pylint: disable when this can be tested
    def _read_into(self, address, buf, length=None):
        # Read a number of bytes from the specified address into the provided
        # buffer.  If length is not specified (the default) the entire buffer
        # will be filled.
        if length is None:
            length = len(buf)
        with self._device as device:
            self._BUFFER[0] = address & 0x7F  # Strip out top bit to set 0
            # value (read).
            device.write(self._BUFFER, end=1)
            device.readinto(buf, end=length)

    def _read_u8(self, address):
        # Read a single byte from the provided address and return it.
        self._read_into(address, self._BUFFER, length=1)
        return self._BUFFER[0]

    def _write_from(self, address, buf, length=None):
        # Write a number of bytes to the provided address and taken from the
        # provided buffer.  If no length is specified (the default) the entire
        # buffer is written.
        if length is None:
            length = len(buf)
        with self._device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF  # Set top bit to 1 to
            # indicate a write.
            device.write(self._BUFFER, end=1)
            device.write(buf, end=length)

    def _write_u8(self, address, val):
        # Write a byte register to the chip.  Specify the 7-bit address and the
        # 8-bit value to write to that address.
        with self._device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF  # Set top bit to 1 to
            # indicate a write.
            self._BUFFER[1] = val & 0xFF
            device.write(self._BUFFER, end=2)

    def reset(self):
        """Perform a reset of the chip."""
        # See section 7.2.2 of the datasheet for reset description.
        self._reset.switch_to_output(value=False)
        time.sleep(0.0001)  # 100 us
        self._reset.switch_to_input(pull=digitalio.Pull.UP)
        time.sleep(0.005)  # 5 ms

    def idle(self):
        """Enter idle standby mode."""
        self.operation_mode = STANDBY_MODE

    def sleep(self):
        """Enter sleep mode."""
        self.operation_mode = SLEEP_MODE

    def listen(self):
        """Listen for packets to be received by the chip.  Use :py:func:`receive`
        to listen, wait and retrieve packets as they're available.
        """
        self.operation_mode = RX_MODE
        self.dio0_mapping = 0b00  # Interrupt on rx done.

    def transmit(self):
        """Transmit a packet which is queued in the FIFO.  This is a low level
        function for entering transmit mode and more.  For generating and
        transmitting a packet of data use :py:func:`send` instead.
        """
        self.operation_mode = TX_MODE
        self.dio0_mapping = 0b01  # Interrupt on tx done.

    @property
    def preamble_length(self):
        """The length of the preamble for sent and received packets, an unsigned
        16-bit value.  Received packets must match this length or they are
        ignored! Set to 8 to match the RadioHead RFM95 library.
        """
        msb = self._read_u8(_RH_RF95_REG_20_PREAMBLE_MSB)
        lsb = self._read_u8(_RH_RF95_REG_21_PREAMBLE_LSB)
        return ((msb << 8) | lsb) & 0xFFFF

    @preamble_length.setter
    def preamble_length(self, val):
        assert 0 <= val <= 65535
        self._write_u8(_RH_RF95_REG_20_PREAMBLE_MSB, (val >> 8) & 0xFF)
        self._write_u8(_RH_RF95_REG_21_PREAMBLE_LSB, val & 0xFF)

    @property
    def frequency_mhz(self):
        """The frequency of the radio in Megahertz. Only the allowed values for
        your radio must be specified (i.e. 433 vs. 915 mhz)!
        """
        msb = self._read_u8(_RH_RF95_REG_06_FRF_MSB)
        mid = self._read_u8(_RH_RF95_REG_07_FRF_MID)
        lsb = self._read_u8(_RH_RF95_REG_08_FRF_LSB)
        frf = ((msb << 16) | (mid << 8) | lsb) & 0xFFFFFF
        frequency = (frf * _RH_RF95_FSTEP) / 1000000.0
        return frequency

    @frequency_mhz.setter
    def frequency_mhz(self, val):
        if val < 240 or val > 960:
            raise RuntimeError("frequency_mhz must be between 240 and 960")
        # Calculate FRF register 24-bit value.
        frf = int((val * 1000000.0) / _RH_RF95_FSTEP) & 0xFFFFFF
        # Extract byte values and update registers.
        msb = frf >> 16
        mid = (frf >> 8) & 0xFF
        lsb = frf & 0xFF
        self._write_u8(_RH_RF95_REG_06_FRF_MSB, msb)
        self._write_u8(_RH_RF95_REG_07_FRF_MID, mid)
        self._write_u8(_RH_RF95_REG_08_FRF_LSB, lsb)

    @property
    def tx_power(self):
        """The transmit power in dBm. Can be set to a value from 5 to 23 for
        high power devices (RFM95/96/97/98, high_power=True) or -1 to 14 for low
        power devices. Only integer power levels are actually set (i.e. 12.5
        will result in a value of 12 dBm).
        The actual maximum setting for high_power=True is 20dBm but for values > 20
        the PA_BOOST will be enabled resulting in an additional gain of 3dBm.
        The actual setting is reduced by 3dBm.
        The reported value will reflect the reduced setting.
        """
        if self.high_power:
            return self.output_power + 5
        return self.output_power - 1

    @tx_power.setter
    def tx_power(self, val):
        val = int(val)
        if self.high_power:
            if val < 5 or val > 23:
                raise RuntimeError("tx_power must be between 5 and 23")
            # Enable power amp DAC if power is above 20 dB.
            # Lower setting by 3db when PA_BOOST enabled - see Data Sheet  Section 6.4
            if val > 20:
                self.pa_dac = _RH_RF95_PA_DAC_ENABLE
                val -= 3
            else:
                self.pa_dac = _RH_RF95_PA_DAC_DISABLE
            self.pa_select = True
            self.output_power = (val - 5) & 0x0F
        else:
            assert -1 <= val <= 14
            self.pa_select = False
            self.max_power = 0b111  # Allow max power output.
            self.output_power = (val + 1) & 0x0F

    @property
    def rssi(self):
        """The received strength indicator (in dBm) of the last received message."""
        # Read RSSI register and convert to value using formula in datasheet.
        # Remember in LoRa mode the payload register changes function to RSSI!
        return self._read_u8(_RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137

    @property
    def signal_bandwidth(self):
        """The signal bandwidth used by the radio (try setting to a higher
        value to increase throughput or to a lower value to increase the
        likelihood of successfully received payloads).  Valid values are
        listed in RFM9x.bw_bins."""
        bw_id = (self._read_u8(_RH_RF95_REG_1D_MODEM_CONFIG1) & 0xF0) >> 4
        if bw_id >= len(self.bw_bins):
            current_bandwidth = 500000
        else:
            current_bandwidth = self.bw_bins[bw_id]
        return current_bandwidth

    @signal_bandwidth.setter
    def signal_bandwidth(self, val):
        # Set signal bandwidth (set to 125000 to match RadioHead Bw125).
        for bw_id, cutoff in enumerate(self.bw_bins):
            if val <= cutoff:
                break
        else:
            bw_id = 9
        self._write_u8(
            _RH_RF95_REG_1D_MODEM_CONFIG1,
            (self._read_u8(_RH_RF95_REG_1D_MODEM_CONFIG1) & 0x0F) | (bw_id << 4),
        )

    @property
    def coding_rate(self):
        """The coding rate used by the radio to control forward error
        correction (try setting to a higher value to increase tolerance of
        short bursts of interference or to a lower value to increase bit
        rate).  Valid values are limited to 5, 6, 7, or 8."""
        cr_id = (self._read_u8(_RH_RF95_REG_1D_MODEM_CONFIG1) & 0x0E) >> 1
        denominator = cr_id + 4
        return denominator

    @coding_rate.setter
    def coding_rate(self, val):
        # Set coding rate (set to 5 to match RadioHead Cr45).
        denominator = min(max(val, 5), 8)
        cr_id = denominator - 4
        self._write_u8(
            _RH_RF95_REG_1D_MODEM_CONFIG1,
            (self._read_u8(_RH_RF95_REG_1D_MODEM_CONFIG1) & 0xF1) | (cr_id << 1),
        )

    @property
    def spreading_factor(self):
        """The spreading factor used by the radio (try setting to a higher
        value to increase the receiver's ability to distinguish signal from
        noise or to a lower value to increase the data transmission rate).
        Valid values are limited to 6, 7, 8, 9, 10, 11, or 12."""
        sf_id = (self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) & 0xF0) >> 4
        return sf_id

    @spreading_factor.setter
    def spreading_factor(self, val):
        # Set spreading factor (set to 7 to match RadioHead Sf128).
        val = min(max(val, 6), 12)
        self._write_u8(_RH_RF95_DETECTION_OPTIMIZE, 0xC5 if val == 6 else 0xC3)
        self._write_u8(_RH_RF95_DETECTION_THRESHOLD, 0x0C if val == 6 else 0x0A)
        self._write_u8(
            _RH_RF95_REG_1E_MODEM_CONFIG2,
            (
                (self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) & 0x0F)
                | ((val << 4) & 0xF0)
            ),
        )

    @property
    def enable_crc(self):
        """Set to True to enable hardware CRC checking of incoming packets.
        Incoming packets that fail the CRC check are not processed.  Set to
        False to disable CRC checking and process all incoming packets."""
        return (self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) & 0x04) == 0x04

    @enable_crc.setter
    def enable_crc(self, val):
        # Optionally enable CRC checking on incoming packets.
        if val:
            self._write_u8(
                _RH_RF95_REG_1E_MODEM_CONFIG2,
                self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) | 0x04,
            )
        else:
            self._write_u8(
                _RH_RF95_REG_1E_MODEM_CONFIG2,
                self._read_u8(_RH_RF95_REG_1E_MODEM_CONFIG2) & 0xFB,
            )

    def send(
        self,
        data,
        timeout=2.0,
        keep_listening=False,
        tx_header=(_RH_BROADCAST_ADDRESS, _RH_BROADCAST_ADDRESS, 0, 0),
    ):
        """Send a string of data using the transmitter.
           You can only send 252 bytes at a time
           (limited by chip's FIFO size and appended headers).
           This appends a 4 byte header to be compatible with the RadioHead library.
           The tx_header defaults to using the Broadcast addresses. It may be overidden
           by specifying a 4-tuple of bytes containing (To,From,ID,Flags)
           The timeout is just to prevent a hang (arbitrarily set to 2 seconds)
           The keep_listening argument should be set to True if you want to start listening
           automatically after the packet is sent. The default setting is False.
        """
        # Disable pylint warning to not use length as a check for zero.
        # This is a puzzling warning as the below code is clearly the most
        # efficient and proper way to ensure a precondition that the provided
        # buffer be within an expected range of bounds. Disable this check.
        # pylint: disable=len-as-condition
        assert 0 < len(data) <= 252
        assert len(tx_header) == 4, "tx header must be 4-tuple (To,From,ID,Flags)"
        # pylint: enable=len-as-condition
        self.idle()  # Stop receiving to clear FIFO and keep it clear.
        # Fill the FIFO with a packet to send.
        self._write_u8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, 0x00)  # FIFO starts at 0.
        # Write header bytes.
        self._write_u8(_RH_RF95_REG_00_FIFO, tx_header[0])  # Header: To
        self._write_u8(_RH_RF95_REG_00_FIFO, tx_header[1])  # Header: From
        self._write_u8(_RH_RF95_REG_00_FIFO, tx_header[2])  # Header: Id
        self._write_u8(_RH_RF95_REG_00_FIFO, tx_header[3])  # Header: Flags
        # Write payload.
        self._write_from(_RH_RF95_REG_00_FIFO, data)
        # Write payload and header length.
        self._write_u8(_RH_RF95_REG_22_PAYLOAD_LENGTH, len(data) + 4)
        # Turn on transmit mode to send out the packet.
        self.transmit()
        # Wait for tx done interrupt with explicit polling (not ideal but
        # best that can be done right now without interrupts).
        start = time.monotonic()
        timed_out = False
        while not timed_out and not self.tx_done:
            if (time.monotonic() - start) >= timeout:
                timed_out = True
        # Listen again if necessary and return the result packet.
        if keep_listening:
            self.listen()
        else:
            # Enter idle mode to stop receiving other packets.
            self.idle()
        # Clear interrupt.
        self._write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)
        if timed_out:
            raise RuntimeError("Timeout during packet send")

    def receive(
        self,
        timeout=0.5,
        keep_listening=True,
        with_header=False,
        rx_filter=_RH_BROADCAST_ADDRESS,
    ):
        """Wait to receive a packet from the receiver. Will wait for up to timeout_s amount of
           seconds for a packet to be received and decoded. If a packet is found the payload bytes
           are returned, otherwise None is returned (which indicates the timeout elapsed with no
           reception).  If timeout is None it is not used ( for use with interrupts)
           If keep_listening is True (the default) the chip will immediately enter listening mode
           after reception of a packet, otherwise it will fall back to idle mode and ignore any
           future reception.
           A 4-byte header must be prepended to the data for compatibilty with the
           RadioHead library.
           The header consists of a 4 bytes (To,From,ID,Flags). The default setting will accept
           any  incomming packet and strip the header before returning the packet to the caller.
           If with_header is True then the 4 byte header will be returned with the packet.
           The payload then begins at packet[4].
           rx_fliter may be set to reject any "non-broadcast" packets that do not contain the
           specfied "To" value in the header.
           if rx_filter is set to 0xff (_RH_BROADCAST_ADDRESS) or if the  "To" field (packet[[0])
           is equal to 0xff then the packet will be accepted and returned to the caller.
           If rx_filter is not 0xff and packet[0] does not match rx_filter then
           the packet is ignored and None is returned.
        """
        timed_out = False
        if timeout is not None:
            # Make sure we are listening for packets.
            self.listen()
            # Wait for the rx done interrupt.  This is not ideal and will
            # surely miss or overflow the FIFO when packets aren't read fast
            # enough, however it's the best that can be done from Python without
            # interrupt supports.
            start = time.monotonic()
            while not timed_out and not self.rx_done:
                if (time.monotonic() - start) >= timeout:
                    timed_out = True
        # Payload ready is set, a packet is in the FIFO.
        packet = None
        if not timed_out:
            if self.enable_crc and self.crc_error:
                warn("CRC error, packet ignored")
            else:
                # Grab the length of the received packet and check it has at least 5
                # bytes to indicate the 4 byte header and at least 1 byte of user data.
                length = self._read_u8(_RH_RF95_REG_13_RX_NB_BYTES)
                if length < 5:
                    packet = None
                else:
                    # Have a good packet, grab it from the FIFO.
                    # Reset the fifo read ptr to the beginning of the packet.
                    current_addr = self._read_u8(_RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR)
                    self._write_u8(_RH_RF95_REG_0D_FIFO_ADDR_PTR, current_addr)
                    packet = bytearray(length)
                    # Read the packet.
                    self._read_into(_RH_RF95_REG_00_FIFO, packet)
                    if (
                        rx_filter != _RH_BROADCAST_ADDRESS
                        and packet[0] != _RH_BROADCAST_ADDRESS
                        and packet[0] != rx_filter
                    ):
                        packet = None
                    elif not with_header:  # skip the header if not wanted
                        packet = packet[4:]
        # Listen again if necessary and return the result packet.
        if keep_listening:
            self.listen()
        else:
            # Enter idle mode to stop receiving other packets.
            self.idle()
        # Clear interrupt.
        self._write_u8(_RH_RF95_REG_12_IRQ_FLAGS, 0xFF)
        return packet
