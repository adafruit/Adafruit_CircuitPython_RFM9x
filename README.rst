Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-rfm9x/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/rfm9x/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

.. image:: https://travis-ci.com/adafruit/Adafruit_CircuitPython_RFM9x.svg?branch=master
    :target: https://travis-ci.com/adafruit/Adafruit_CircuitPython_RFM9x
    :alt: Build Status

CircuitPython module for the RFM95/6/7/8 LoRa 433/915mhz radio modules.

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Usage Example
=============

Initialization of the RFM radio requires specifying a frequency appropriate to
your radio hardware (i.e. 868-915 or 433 MHz) and specifying the pins used in your
wiring from the controller board to the radio module.

This example code matches the wiring used in the
`LoRa and LoRaWAN Radio for Raspberry Pi <https://learn.adafruit.com/lora-and-lorawan-radio-for-raspberry-pi/>`_
project:

.. code-block:: python

    import digitalio
    import board
    import busio
    import adafruit_rfm9x

    RADIO_FREQ_MHZ = 915.0
    CS = digitalio.DigitalInOut(board.CE1)
    RESET = digitalio.DigitalInOut(board.D25)
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

Note: the default baudrate for the SPI is 50000000 (5MHz). The maximum setting is 10Mhz but 
transmission errors have been observed expecially when using breakout boards.
For breakout boards or other configurations where the boards are separated, it may be necessary to reduce
the baudrate for reliable data transmission.
The baud rate may be specified as an keyword parameter when initializing the board.
To set it to 1000000 use :

.. code-block:: python

    # Initialze RFM radio with a more conservative baudrate
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)

Optional controls exist to alter the signal bandwidth, coding rate, and spreading factor
settings used by the radio to achieve better performance in different environments.
By default, settings compatible with RadioHead Bw125Cr45Sf128 mode are used, which can
be altered in the following manner (continued from the above example):

.. code-block:: python

    # Apply new modem config settings to the radio to improve its effective range
    rfm9x.signal_bandwidth = 62500
    rfm9x.coding_rate = 6
    rfm9x.spreading_factor = 8
    rfm9x.enable_crc = True

See examples/rfm9x_simpletest.py for an expanded demo of the usage.


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_RFM9x/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

Zip release files
-----------------

To build this library locally you'll need to install the
`circuitpython-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install circuitpython-build-tools

Once installed, make sure you are in the virtual environment:

.. code-block:: shell

    source .env/bin/activate

Then run the build:

.. code-block:: shell

    circuitpython-build-bundles --filename_prefix adafruit-circuitpython-rfm9x --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.
