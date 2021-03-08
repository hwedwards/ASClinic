.. _comm-protocol-i2c:

I2C (Inter-Integrated Circuit)
==============================

A single pin GPIO has only two states, ``0`` and ``1``, which can be either driven by the computer associated with the GPIO pin, or driven by an external device connected to the pin. The timing of driving changes between the two states can be arbitary up to the resolution of the microcontroller that controls the GPIO.



Install the libi2c-dev library. In order to be able inspect the LED Display, you may find it useful to also install the i2c tools:

sudo apt-get install libi2c-dev i2c-tools

After installation, in a Terminal execute (1 is the I2C bus in this case):

sudo i2cdetect -y -r 1



``i2c_msg`` struct definition found here:

* https://docs.huihoo.com/doxygen/linux/kernel/3.7/structi2c__msg.html

and the code definition is found in the ``i2c.h`` file:

* https://docs.huihoo.com/doxygen/linux/kernel/3.7/include_2uapi_2linux_2i2c_8h_source.html

The source code for ``i2c_tools`` is published here:

* https://git.kernel.org/pub/scm/utils/i2c-tools/i2c-tools.git/
* https://git.kernel.org/pub/scm/utils/i2c-tools/i2c-tools.git/about/

Similar by different documention is found here:

* https://www.kernel.org/doc/Documentation/i2c/dev-interface

A wiki for ``i2c_tools`` is found here:

* https://i2c.wiki.kernel.org/index.php/I2C_Tools


I2C Addresses
*************

The following table lists the possible I2C address for the various sensor and actuator boards provided. The possible address are separated into three columns:

* **Default**: the address of the component when hooked-up out-of-the-box.
* **Hardware Options**: the possible addresses that can be configured by a hardware modification, usually by connecting or cutting pads on the breakout board.
* **Software Options**: the possible addresses the can be configured through some form of software interface.

.. list-table:: I2C Addresses per component
   :widths: 40 20 20 20
   :width: 100
   :header-rows: 1
   :stub-columns: 1
   :align: center

   * - Component
     - Default Address
     - Hardware Options
     - Software Options
   * - Pololu Simple Motor

       Controller
     - 0x0D (13)
     - None
     - Any :superscript:`1`
   * - BMM150

       Magnetometer
     - 0x10 (16)
     - 0x11 - 0x13

       (17-19)
     - None
   * - BMI088

       Accelerometer
     - 0x18 (24)
     - 0x19 (25)
     - None
   * - VL53L1X Laser

       Distance Sensor
     - 0x29 (41)
     - None
     - None
   * - TCS34725 RGB

       Colour Sensor
     - 0x29 (41)
     - None
     - None
   * - INA260 Current

       Sensor
     - 0x40 (64)
     - 4 LBSs

       0x41 - 0x4F

       (64 - 79)
     - None
   * - PCA9685 16-Channel

       Servo Driver
     - 0x40 (64)
     - 6 LBSs

       0x41 - 0x7F

       (64 - 127)
     - None
   * - PCA9685 All Call

       I2C Bus Address
     - 0x70 (112)
     - None
     - Any :superscript:`2`
   * - BMI088

       Gyroscope
     - 0x68 (104)
     - 0x69 (105)
     - None
   * - ICM-20948 IMU
     - 0x69 (105)
     - 0x68 (104)
     - None
   * - BMP390 Pressure

       Sensor
     - 0x77 (119)
     - 0x76 (118)
     - None

:superscript:`1` Pololu provides a `Windows based software <https://www.pololu.com/docs/0J77/3.1>`_ that is used for setting the software defined I2C address.

:superscript:`2` This is a software defined address of the **PCA9685** chip, however, it is enable on power-up with the default address indicated in the table. Hence the `PCA9685 documentation <https://www.nxp.com/products/power-management/lighting-driver-and-controller-ics/ic-led-controllers/16-channel-12-bit-pwm-fm-plus-ic-bus-led-controller:PCA9685>`_ provides the following remark: *The default LED All Call I2C-bus address (70h or 0111 0000) must not be used as a regular I2C-bus slave address since this address is enabled at power-up*


..
  sudo i2cdetect -y -r 1
  0 1 2 3 4 5 6 7 8 9 a b c d e f
  00: — — — — — — — — — — — — —
  10: — — — — — — — — — — — — — — — —
  20: — — — — — — — — — — — — — — — —
  30: — — — — — — — — — — — — — — — —
  40: — — — — — — — — — — — — — — — —
  50: — — — — — — — — — — — — — — — —
  60: — — — — — — — — — — — — — — — —
  70: 70 — — — — — — —
