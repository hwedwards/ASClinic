.. _sbc-jetson-xavier-nx:

Jetson Xavier NX Developer Kit
==============================

This :ref:`single board computer <single-board-computers>` is used as main computer for the robot.


Documentation links
*******************

The most relevant documentation for the details of the Jetson Xavier NX dev kit are:

* `Xavier NX Dev Kit User Guide <https://developer.nvidia.com/embedded/downloads#?search=developer%20kit%20user%20guide&tx=$product,jetson_xavier_nx>`_
* `Xavier NX Dev Kit Board Specification <https://developer.nvidia.com/embedded/downloads#?search=board%20specification&tx=$product,jetson_xavier_nx>`_
* `Xavier NX Module Data Sheet <https://developer.nvidia.com/embedded/downloads#?search=module%20data%20sheet&tx=$product,jetson_xavier_nx>`_


40-pin expansion header
***********************

The 40-pin expansion header provides access to the I2C bus and GPIO pins of the Jetson.

Pin layout
^^^^^^^^^^

The layout of the 40-pin J12 Expansion header is found in the `Xavier NX Dev Kit Board Specification <https://developer.nvidia.com/embedded/downloads#?search=board%20specification&tx=$product,jetson_xavier_nx>`_.
An alternative visualisation of the layout is given `here on the Jetson hacks website <https://www.jetsonhacks.com/nvidia-jetson-xavier-nx-gpio-header-pinout/>`_.


Configuring the function of each pin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`This guide <https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/hw_setup_jetson_io.html>`_ provides comprehensive instructions for using the python tools provided by Nvidia for configuring the 40-pin expansion header.

The terminal-based interactive tool for configuring the 40-pin expansion header is called :code:`jetson-io` and can be launched with the following command:

.. code-block:: bash

	sudo /opt/nvidia/jetson-io/jetson-io.py


Additionally, three command line tools are provided to display information about, and configure the pins of, the 40-pin expansion header. The three command line tools are:

.. code-block:: bash

	sudo /opt/nvidia/jetson-io/config-by-pin.py
	sudo /opt/nvidia/jetson-io/config-by-function.py
	sudo /opt/nvidia/jetson-io/config-by-hardware.py

All three commands can be passed the option :code:`-h` or :code:`--help` to display the information about how to use the command line tool.

List the current configuration using:

.. code-block:: bash

	sudo /opt/nvidia/jetson-io/config-by-pin.py


.. warning:: Do **NOT** choose a :code:`tegra-gpio` line at random for your GPIO use case, even if it is listed as :code:`unused`. Only use lines that are documented as mapping to a specific pin on the J12 40-pin expansion header.


.. _sbc-jetson-xavier-nx-pin-mapping:

Mapping pin number to line number
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A mapping from pin-number on the J12 40-pin expansion header to the line-number is also given on `this page on the Jetson hacks website <https://www.jetsonhacks.com/nvidia-jetson-xavier-nx-gpio-header-pinout/>`_.

The following table summarises the mapping:
- from the pin-number of the J12 expansion header,
- to the default configuration of the pin,
- to the Linux GPIO line-number within the Tegra chip (i.e., with :code:`gpiochip0` or equivalently :code:`tegra-gpio`).

.. code-block::

	|-------|----------------|---------|----------------|-------|
	| Tegra |                |   J12   |                | Tegra |
	| Line  | Default        |   Pin   | Default        | Line  |
	|-------|----------------|---------|----------------|-------|
	|       | 3.3V Supply    |  1 |  2 | 5.0V Supply    |       |
	|       | I2C1 SDA       |  3 |  4 | 5.0V Supply    |       |
	|       | I2C1 SCL       |  5 |  6 | Ground         |       |
	|   148 | GPIO 09        |  7 |  8 | UART1_TX       |       |
	|       | Ground         |  9 | 10 | UART1_RX       |       |
	|   140 | UART1_RTS      | 11 | 12 | I2S0_SCLK      |   157 |
	|   192 | SPI1_SCK       | 13 | 14 | Ground         |       |
	|  ^ 20^| GPIO 12        | 15 | 16 | SPI1_CS1       |   196 |
	|       | 3.3V Supply    | 17 | 18 | SPI1_CS0       |   195 |
	|   205 | SPI0_MOSI      | 19 | 20 | Ground         |       |
	|   204 | SPI0_MOSO      | 21 | 22 | SPI1_MISO      |   193 |
	|   203 | SPI0_SCK       | 23 | 24 | SPI0_CS0       |   206 |
	|       | Ground         | 25 | 26 | SPI0_CS0       |   207 |
	|       | I2C0 SDA       | 27 | 28 | I2C0 SCL       |       |
	|   133 | GPIO 01        | 29 | 30 | Ground         |       |
	|   134 | GPIO 11        | 31 | 32 | GPIO 07        |   136 |
	|   105 | GPIO 13        | 33 | 34 | Ground         |       |
	|   160 | I2S_FS         | 35 | 36 | UART1_CTS      |   141 |
	|   194 | SPI1_MOSI      | 37 | 38 | I2S0_SDIN      |   159 |
	|       | Ground         | 39 | 40 | I2S0_SDOUT     |   158 |
	|-------|----------------|---------|----------------|-------|


.. note::
	Pins (3,5) correspond to I2C bus 8, and pins (27,28) correspond to I2C bus 1. Hence the connected devices can be quickly checked from command line using: :code:`sudo i2cdetect -y -r <bus_number>` where :code:`<bus_number>` is replaced by 8 or 1.

..
	.. note:: the :code:`Tegra line` numbers marked with exclamation marks, i.e., of the form :code:`!xxx!`, should **NOT** be used as GPIO pins. The information displayed by :code:`sudo gpioinfo tegra-gpio` lists these lines as :code:`unused`, but they should still **NOT** be used as GPIO pins

.. note:: the :code:`Tegra line` numbers marked with hats, i.e., of the form :code:`^xxx^`, did not work when tested without additional configuration..
