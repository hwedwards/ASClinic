.. _comm-protocol-GPIO:

Single Pin GPIO (General Purpose Input Output)
==============================================

A single pin GPIO has only two states, :code:`0` and :code:`1`, which can be either driven by the computer associated with the GPIO pin, or driven by an external device connected to the pin. The timing of driving changes between the two states can be arbitary up to the resolution of the microcontroller that controls the GPIO.


The :code:`gpiod` library
*************************

For interfacing with the GPIO pins, we use the library :code:`libgpiod`, which stands for:

:code:`lib`\_rary for :code:`g`\_eneral :code:`p`\_urpose :code:`i`\_nput/:code:`o`\_utput :code:`d`\_evices

For an Ubuntu operating system, this library can be is installed via:

.. code-block:: bash

  sudo apt install gpiod

The most official page of the :code:`libgpiod` library can be found here:

  * https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/about/

The following provide some helpful discussion about :code:`libgpiod`:

* `PDF of slides <https://ostconf.com/system/attachments/files/000/001/532/original/Linux_Piter_2018_-_New_GPIO_interface_for_linux_userspace.pdf?1541021776>`_
* Recorded presentation: https://youtu.be/BK6gOLVRKuU

Other libraries considered were :code:`sysfs`, :code:`WiringPi`, and :code:`jetson-gpio`:

* https://blog.adafruit.com/2018/11/26/sysfs-is-dead-long-live-libgpiod-libgpiod-for-linux-circuitpython/
* http://wiringpi.com/
* https://github.com/NVIDIA/jetson-gpio



Using :code:`gpiod` in C++ code
*******************************

The following two websites both provide the most comprehensive list of all the C++ functions offered by the :code:`gpiod` library for monitoring a GPIO pin:

  * `gpiod documentation hosted by dlang <https://libgpiod-dlang.dpldocs.info/gpiod.html>`_
  * `gpiod doxygen documentation hosted by lane-fu <https://www.lane-fu.com/linuxmirror/libgpiod/doc/html/index.html>`_

The following compiler flag is necessary for any code using the :code:`gpiod` library functions:

  * :code:`-lgpiodcxx` for C++ code
  * :code:`-lgpiod` for C code 


Command line tools from :code:`gpiod`
*************************************

The command line tools provided by :code:`libgpiod` are:

  * :code:`gpiodetect` list all gpiochips present on the system, their names, labels and number of GPIO lines
  * :code:`gpioinfo` list all lines of specified gpiochips, their names, consumers, direction, active state and additional flags
  * :code:`gpioget` read values of specified GPIO lines
  * :code:`gpioset` set values of specified GPIO lines, potentially keep the lines exported and wait until timeout, user input or signal
  * :code:`gpiofind` find the gpiochip name and line offset given the line name
  * :code:`gpiomon` wait for events on GPIO lines, specify which events to watch, how many events to process before exiting or if the events should be reported to the console

For the Jetson TX2 Developer Kit, the command:

.. code-block:: bash

  sudo gpiodetect


should display the following:

.. code-block:: bash

  gpiochip0 [tegra-gpio] (192 lines)
  gpiochip1 [tegra-gpio-aon] (64 lines)
  gpiochip2 [tca9539] (16 lines)
  gpiochip3 [tca9539] (16 lines)
  gpiochip4 [max77620-gpio] (8 lines)

All the GPIO line of the J21 40-pin expansion header are connected into the :code:`tegra-gpio`, i.e., into :code:`gpiochip0`.

To display the details of the 192 `tegra-gpio` line, use either of the following commands:
```
sudo gpioinfo gpiochip0
sudo gpioinfo tegra-gpio
```
This should list most of the lines as `unnamed` `unused` `input` `active-high`.


Test that a GPIO input is working
*********************************

The :code:`gpioget` and :code:`gpiomon` command line tools can be used to perform a quick test that a input to a GPIO pin is properly connected and functioning as expected.

To read the value, :code:`{0,1}`, from the GPIO pin, used the command:

.. code-block:: bash

  sudo gpioget <gpio_chip_name> <line_number>

For example, if the input is connected to pin 18 of the J21 expansion header, then according to the table above pin 18 connects to line 161 of the :code:`tegra-gpio` chip. Hence you can read the value of pin 18 with the

.. code-block:: bash

  sudo gpioget tegra-gpio 161

To monitor a line for multiple of a particular event, use the following:

.. code-block:: bash

  sudo gpiomon --num-events=3 --rising-edge tegra-gpio 161
