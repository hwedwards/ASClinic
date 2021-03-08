.. _sbc-jetson-tx2:

Jetson TX2 Developer Kit
========================

This :ref:`single board computer <single-board-computers>` was initially considered for the main computer of the robot and was used for initial prototyping. But during the development phase, the Jetson Xavier NX was released as the successor to the TX2.


Documentation links
*******************

The most relevant documentation for the details of the Jetson TX2 dev kit are:

* `TX2 Dev Kit User Guide <https://developer.nvidia.com/embedded/downloads#?search=developer%20kit%20user%20guide&tx=$product,jetson_tx2>`_
* `TX2 Dev Kit Board Specification <https://developer.nvidia.com/embedded/downloads#?search=board%20specification&tx=$product,jetson_tx2>`_
* `TX2 Module Data Sheet <https://developer.nvidia.com/embedded/downloads#?search=module%20data%20sheet&tx=$product,jetson_tx2>`_


40-pin expansion header
***********************

The 40-pin expansion header provides access to the I2C bus and GPIO pins of the Jetson.

Pin layout
^^^^^^^^^^

The layout of the 40-pin J21 Expansion header is found in the `TX2 Dev Kit User Guide <https://developer.nvidia.com/embedded/downloads#?search=developer%20kit%20user%20guide&tx=$product,jetson_tx2>`_.
An alternative visualisation of the layout is given `here on the Jetson hacks website <https://www.jetsonhacks.com/nvidia-jetson-tx2-j21-header-pinout/>`_.


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


.. warning:: Do **NOT** choose a :code:`tegra-gpio` line at random for your GPIO use case, even if it is listed as :code:`unused`. Only use lines that are documented as mapping to a specific pin on the J21 40-pin expansion header.


Mapping pin number to line number
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A spreadsheet that provides the mapping from pin-number on the J21 40-pin expansion header to the line-number can be found at the bottom of `this page on the Jetson hacks website <https://www.jetsonhacks.com/nvidia-jetson-tx2-j21-header-pinout/>`_.

The following table summarises the mapping:
- from the pin-number of the J21 expansion header,
- to the default configuration of the pin,
- to the Linux GPIO line-number within the Tegra chip (i.e., with :code:`gpiochip0` or equivalently :code:`tegra-gpio`).

.. code-block::

	|-------|----------------|---------|----------------|-------|
	| Tegra |                |   J21   |                | Tegra |
	| Line  | Default        |   Pin   | Default        | Line  |
	|-------|----------------|---------|----------------|-------|
	|       | 3.3V Supply    |  1 |  2 | 5.0V Supply    |       |
	|   !49!| I2C1 SDA 3.3V  |  3 |  4 | 5.0V Supply    |       |
	|   !48!| I2C1 SCL 3.3V  |  5 |  6 | Ground         |       |
	|    76 | GPIO 1.8or3.3V |  7 |  8 | UART TXD 3.3V  |  !144!|
	|       | Ground         |  9 | 10 | UART RXD 3.3V  |  !145!|
	|  ^146^| GPIO 3.3V      | 11 | 12 | GPIO 1.8or3.3V |    72 |
	|    77 | GPIO 1.8or3.3V | 13 | 14 | Ground         |       |
	|   ^15^| GPIO 3.3V      | 15 | 16 | GPIO 1.8or3.3V |   ^40^|
	|       | 3.3V Supply    | 17 | 18 | GPIO 1.8or3.3V |   161 |
	|   109 | GPIO 1.8or3.3V | 19 | 20 | Ground         |       |
	|   108 | GPIO 1.8or3.3V | 21 | 22 | GPIO 3.3V      |   ^14^|
	|   107 | GPIO 1.8or3.3V | 23 | 24 | GPIO 1.8or3.3V |   110 |
	|       | Ground         | 25 | 26 | Not Used       |       |
	|   !22!| I2C0 SDA 3.3V  | 27 | 28 | I2C0 SCL 3.3V  |   !21!|
	|    78 | GPIO 1.8or3.3V | 29 | 30 | Ground         |       |
	|   ^42^| GPIO 3.3V      | 31 | 32 | GPIO 1.8or3.3V |   ^41^|
	|    69 | GPIO 1.8or3.3V | 33 | 34 | Ground         |       |
	|    75 | GPIO 1.8or3.3V | 35 | 36 | GPIO 3.3V      |   147 |
	|    68 | GPIO 3.3V      | 37 | 38 | GPIO 1.8or3.3V |    74 |
	|       | Ground         | 25 | 26 | GPIO 1.8or3.3V |    73 |
	|-------|----------------|---------|----------------|-------|


.. note:: the :code:`Tegra line` numbers marked with exclamation marks, i.e., of the form :code:`!xxx!`, should **NOT** be used as GPIO pins. The information displayed by :code:`sudo gpioinfo tegra-gpio` lists these lines as :code:`unused`, but they should still **NOT** be used as GPIO pins

.. note:: the :code:`Tegra line` numbers marked with hats, i.e., of the form :code:`^xxx^`, did not work when tested without additional configuration.
