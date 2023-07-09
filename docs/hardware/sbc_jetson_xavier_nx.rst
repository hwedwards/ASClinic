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

Mapping pin number to line number for :code:`gpiod`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The line number that is expected for :ref:`comm-protocol-GPIO-gpiod-library` is something that can change with the kernel, i.e., it is software defined. Hence it can change from version-to-version of the Nvidia JetPack operating system. For example, this did change in going from Jetpack 4.x to Jetpack 5.x.

The mapping here tabulates the physical connection between the physical pins on the 40-pin header of the Developer Kit carrier board and the physical pins/port of the Xavier NX module (also-known-as the System on a Chip or "SoC"). Using the port number given in the table below, you can find the line number required for the :code:`gpiod` library by running the following command in a terminal on the Jetson:

.. code-block:: bash

  sudo gpiofind "<port number>"

Where :code:`<port number>` is replaced by the string in the table below, for example, the command:

.. code-block:: bash

  sudo gpiofind "PQ.05"

displays the chip number and line number, for example, on Jetpack 5.x this displays:

.. code-block:: bash

  gpiodchip1 line 105

In other words, for physical pin number 29 of the 40-pin J12 header on the Dev Kit carrier board, the table below indicates that this is port "PQ.05", and the :code:`gpiofind` command output indicates that this is line 105 of :code:`gpiochip1`. You can double check this is correct from command line by using the following monitoring command:

.. code-block:: bash

  sudo gpiomon --num-events=3 --rising-edge gpiochip1 105

and then providing some pulses on physical pin number 29 of the 40-pin header.

The following table summarises the mapping:

* From the physical pin-number of the J12 expansion header:
* To the so-called port number of the Jetson Xavier NX (SoC) module;
* To the default name of the pin;
* To the physical pin number of the Jetson Xavier NX (SoC) module;

The data source for this mapping is the `NVIDIA Jetson Xavier NX Developer Kit Carrier Board Specification (P3509_A01) <https://developer.nvidia.com/jetson-xavier-nx-developer-kit-carrier-board-specification-p3509-a01>`_. In case this link does not work, simply go to the `Nvidia Jetson Download Center <https://developer.nvidia.com/embedded/downloads>`_ and filter for "Jetson Xavier NX" downloads.


.. code-block::

  |-----|-------|-------------|---------|-------------|-------|-----|
  | SoC | SoC   | Default     |   J12   | Default     | SoC   | SoC |
  | Pin | Port  | Name        |   Pin   | Name        | Port  | Pin |
  |-----|-------|-------------|---------|-------------|-------|-----|
  |     |       | 3.3V Supply |  1 |  2 | 5.0V Supply |       |     |
  | 191 |       | I2C1_SDA    |  3 |  4 | 5.0V Supply |       |     |
  | 189 |       | I2C1_SCL    |  5 |  6 | Ground      |       |     |
  | 211 | PS.04 | GPIO09      |  7 |  8 | UART1_TXD   | PR.02 | 203 |
  |     |       | Ground      |  9 | 10 | UART1_RXD   | PR.03 | 205 |
  | 207 | PR.04 | UART1_RTS   | 11 | 12 | I2S0_SCLK   | PT.05 | 199 |
  | 106 | PY.00 | SPI1_SCK    | 13 | 14 | Ground      |       |     |
  | 218 | PCC.04| GPIO12      | 15 | 16 | SPI1_CSI1   | PY.04 | 112 |
  |     |       | 3.3V Supply | 17 | 18 | SPI1_CSI0   | PY.03 | 110 |
  |  89 | PZ.05 | SPI0_MOSI   | 19 | 20 | Ground      |       |     |
  |  93 | PZ.04 | SPI0_MOSO   | 21 | 22 | SPI1_MISO   | PY.01 | 108 |
  |  91 | PZ.03 | SPI0_SCK    | 23 | 24 | SPI0_CS0    | PZ.06 |  95 |
  |     |       | Ground      | 25 | 26 | SPI0_CS0    | PZ.07 |  97 |
  | 187 | PDD.00| I2C0_SDA    | 27 | 28 | I2C0_SCL    | PCC.07| 185 |
  | 118 | PQ.05 | GPIO01      | 29 | 30 | Ground      |       |     |
  | 216 | PQ.06 | GPIO11      | 31 | 32 | GPIO07      | PR.00 | 206 |
  | 228 | PN.01 | GPIO13      | 33 | 34 | Ground      |       |     |
  | 197 | PU.00 | I2S_FS      | 35 | 36 | UART1_CTS   | PR.05 | 209 |
  | 104 | PY.02 | SPI1_MOSI   | 37 | 38 | I2S0_DIN    | PT.07 | 195 |
  |     |       | Ground      | 39 | 40 | I2S0_DOUT   | PT.06 | 193 |
  |-----|-------|-------------|---------|-------------|-------|-----|


.. note::

  Pins (3,5) correspond to I2C bus 8, and pins (27,28) correspond to I2C bus 1. Hence the connected devices can be quickly checked from command line using:

  .. code-block:: bash

    sudo i2cdetect -y -r <bus_number>

  where :code:`<bus_number>` is replaced by 8 or 1.


.. note::

  For a differential-drive robot with quadrature encoders on each wheel, we recommend connecting the 4 encoder wires to physical pins {29, 31, 33, 35} on the 40-pin J12 header of the Dev Kit carrier board, which corresponds to the following :code:`gpiod` chip and line numbers:

  * For Jetpack 4.x: :code:`gpiochip0`, line numbers {133, 134, 105, 160}
  * For Jetpack 5.x: :code:`gpiochip1`, line numbers {105, 106, 84, 130}


The following are two additional sources of information about the GPIO pin mappings:

* View the file :code:`/sys/kernel/debug/gpio` for a mapping from SoC Port number to :code:`sysfs` GPIO number.
* Display the line ranges of the GPIO chips using the command:

  .. code-block:: bash

    sudo dmesg | grep gpiochip

* View the conversation details from SoC Port number to :code:`gpiod` line number in the header file :code:`tegra194-gpio.h`, which you can locate with the command:

  .. code-block:: bash

    sudo find /usr tegra194-gpio.h

* Download and view the `Jetson Xavier NX Pinmux Table <https://developer.nvidia.com/jetson-xavier-nx-pinmux-configuration-template-v106>`_, which is available from the `Nvidia Jetson Download Center <https://developer.nvidia.com/embedded/downloads>`_.



..
  Mapping pin number to line number for Jetack 4.x
  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  .. note::
    The line number that is expected for :ref:`comm-protocol-GPIO-gpiod-library` is something that can change with the kernel, i.e., it is software defined.

    The section above provide a general method to check the line number for the software running on your Jetson. This section is kept for legacy reasons.


  A mapping from pin-number on the J12 40-pin expansion header to the line-number is given on `this page on the Jetson hacks website <https://www.jetsonhacks.com/nvidia-jetson-xavier-nx-gpio-header-pinout/>`_.

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
