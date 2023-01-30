.. _drivers-cpp-i2c:

I2C Driver for CPP
==================

This driver is a C++ class for abstracting I2C write and read commands on a Linux operating system.

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2


Class definition
****************

.. cpp:class:: I2C_Driver

  This class uses the :code:`ioctl` user-space framework provided by Linux for interfacing with an I2C bus. The key includes to enable this system functionality are:

  * :code:`#include <fcntl.h>`
  * :code:`#include <linux/i2c.h>`
  * :code:`#include <linux/i2c-dev.h>`
  * :code:`#include <sys/ioctl.h>`

  More information about :code:`ioctl` can be found here in the `I2C device interface documentation of the Linux kernel <https://www.kernel.org/doc/html/latest/i2c/dev-interface.html>`_. If you encounter errors arising from the :code:`ioctl` function, this `blog post discuss interpretting of the errno <https://stackoverflow.com/questions/503878/how-to-know-what-the-errno-means>`_.


..
  Rather than set the global namespace like this:
  .. cpp:namespace:: I2C_Driver
  We choose to define the namespace explicitly for every item below.

  Hence empty the global namespace in case it is set elsewhere:

.. cpp:namespace:: nullptr


Enumerations
************

.. cpp:enum-class:: I2C_Driver::I2C_State : int

   A scoped enum for this class that indicates that current state of the I2C connection as either `open` or `closed`.

Member variables
****************

.. cpp:member:: private char I2C_Driver::m_device_name[20]

  The name of the I2C device that this class interfaces with. For a Linux operating system, the device names are almost always of the form :code:`/dev/i2c-1`, where the number at the end of the name indicates the specific I2C bus number.

  The maximum length of 20 characters for this variable is arbitrarily chosen to be sufficiently longer than any plausible device name.

.. cpp:member:: private I2C_Driver::I2C_State I2C_Driver::m_state

  The current state of the I2C device that this instance manages. As per :cpp:enum:`I2C_State`, the state can be either open or closed.

.. cpp:member:: private int I2C_Driver::m_file_descriptor

  File descriptor of the I2C connection as returned by the operating system when the I2C connection is open. This variable is set to :code:`-1` if the connection is closed.

Class constructor
*****************

.. cpp:function:: I2C_Driver::I2C_Driver(const char * device_name)

   Class constructor that set the device name variable :cpp:member:`m_device_name`. It is not possible to change the device name after construction of the object, you instead need to create a new object with the different device name.


Getter functions
****************

.. cpp:function:: public const char * I2C_Driver::get_device_name()

  Returns a pointer to the start of the device name variable :cpp:member:`m_device_name`.

.. cpp:function:: public int I2C_Driver::get_state()

  Returns the value of the connection state variable :cpp:member:`m_state` as an integer.

.. cpp:function:: int I2C_Driver::get_file_descriptor()

  Returns the value of the file descriptor variable :cpp:member:`m_file_descriptor`, which is set to :code:`-1` if the connection state is closed.


Member functions
****************

.. cpp:function:: public bool I2C_Driver::open_i2c_device()

  Attempts to open the I2C device with name :cpp:member:`m_device_name`. Uses the :code:`open()` function provided by the :code:`<fcntl.h>` header and specifies the mode as :code:`0_RDWR`, i.e., read-write mode.

  | **Returns: bool**
  |   Boolean flag indicating the status.
  |   :code:`true` indicates that a valid file descriptor was provided and hence the I2C device is open.
  |   :code:`false` indicates that I2C device could not be opened, hence the file descriptor is :code:`-1`.

.. cpp:function:: public bool I2C_Driver::close_i2c_device()

  Closes the I2C device that corresponds to the value of the :cpp:member:`m_file_descriptor` variable. Uses the :code:`close()` function provided by the :code:`<fcntl.h>` header.

  | **Returns: bool**
  |   Boolean flag indicating the status.
  |   :code:`true` indicates that the device designated by :cpp:member:`m_file_descriptor` was successfully closed, and hence :cpp:member:`m_file_descriptor` was set to :code:`-1`.
  |   :code:`false` indicates that there was not an I2C device to close because :cpp:member:`m_file_descriptor` has the value :code:`-1`.

.. cpp:function:: public bool I2C_Driver::write_data(uint8_t address, uint16_t num_write_btyes, uint8_t * write_data_array)

  Writes the specified device address on the the I2C bus with the write bit set, and then write the data provided onto the bus, hence sending that data to that device.

  | **Parameters:**
  |   **uint8_t address**
  |   I2C address of the device to communicate with.

    | **uint16_t num_write_btyes**
    | Number of bytes of data to be written.

    | **uint8_t * write_data_array**
    | Pointer to the array that contains the data to be written.

  | **Returns: bool**
  |   Boolean flag indicating the status of the I2C write, :code:`true` for successful, :code:`false` otherwise.


.. cpp:function:: public bool I2C_Driver::write_data_then_read_data(uint8_t address, uint16_t num_write_btyes, uint8_t * write_data_array, uint16_t num_read_btyes, uint8_t * read_data_array)

  Writes the specified device address on the the I2C bus with the write bit set, and then write the data provided onto the bus, hence sending that data to that device. Subsequently listens for and reads the number of bytes specified.

  | **Parameters:**
  |   **uint8_t address**
  |   I2C address of the device to communicate with.

    | **uint16_t num_write_btyes**
    | Number of bytes of data to be written.

    | **uint8_t * write_data_array**
    | Pointer to the array that contains the data to be written.

    | **uint16_t num_read_btyes**
    | Number of bytes of data to be read.

    | **uint8_t * read_data_array**
    | Pointer to the array where the read data is stored.

  | **Returns: bool**
  |   Boolean flag indicating the status of the I2C write-then-read, :code:`true` for successful, :code:`false` otherwise.


.. cpp:function:: public bool I2C_Driver::check_for_device_at_address(uint8_t address)

  Checks whether or not there is a device connected at the specified address.

  There is no standard I2C detection command or function. The method used by this function to write one byte with value :code:`0x00` to the specified address and then attempt to read one byte worth of response.

  .. important::

    This detection method works well for most devices. However, is not guaranteed to work for all devices and can in some circumstances it cause unexpected behaviour (see `Ubuntu manual page for i2cdetect <https://manpages.ubuntu.com/manpages/bionic/man8/i2cdetect.8.html>`_ for further information).

  | **Parameters:**
  |   **uint8_t address**
  |   I2C address to check whether there is a device currently connected.

  | **Returns: bool**
  |   Boolean flag indicating the status for that address.
  |   :code:`true` indicates that a byte of data was receive in response to probing the address, hence that is a device connected at the address.
  |   :code:`false` indicates that no response was received, hence it is unlikley for there to be a device connected at the address.
