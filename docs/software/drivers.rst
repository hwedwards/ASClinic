.. _drivers:

SOFTWARE DRIVERS FOR SENSORS AND ACTUATORS
==========================================

.. toctree::
  :maxdepth: 2
  :hidden:

  drivers_cpp_i2c
  drivers_cpp_pololu_smc
  
Many sensors and actuators operate using communication protocols that runs over the GPIO pins, for example, I2C and SPI. These sensors and actuators generally require a software driver running on the robot that converts our "human readable" requests into the string of bit that needs to be written-to and read-from the pins of the serial interface.

This section of the wiki documents the class that we have developed for:

* :ref:`drivers-cpp-i2c`: a C++ class for abstracting I2C write and read commands on a Linux operating system.

* :ref:`drivers-cpp-pololu-smc`: a C++ class for interfacing with the `"Simple Motor Controller (SMC)" boards produced Pololu <https://www.pololu.com/category/94/pololu-simple-motor-controllers>`_.

* `drivers-cpp-servo-pca9685`: a C++ class for interfacing with the 16-channel 12-bit Pulse Width Modulation (PWM) generator `PCA9685 chip produced NXP Semiconductors <https://www.nxp.com/products/power-management/lighting-driver-and-controller-ics/led-controllers/16-channel-12-bit-pwm-fm-plus-ic-bus-led-controller:PCA9685>`_ and used on breakout boards from Adafruit, Sparkfun, OpenMV, etc.
