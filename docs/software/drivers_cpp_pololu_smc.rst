.. _drivers-cpp-pololu-smc:

I2C Driver for Pololu Simple Motor Controller (SMC)
===================================================

This driver is a C++ class for abstracting the functions of the Pololu Simple Motor Controller (SMC) G2 boards. Full documentation of the board's I2C commands are given in the `Pololu SMC G2 user's guide <https://www.pololu.com/docs/0J77>`_.

.. note::

  The Pololu SMC G2 board is not "controller" in the sense that it does not have any feedback measurements about the rotational speed of the motor, and hence it does not control the rotational speed.

  It is a "controller" in the terminology of distinguishing between:

  * A motor **driver** board, which accepts PWM input signals and use a H-bridge architecture to send power to the motor.
  * A motor **controller**, which accepts scalar duty cycle commands over some interface and generates the PWM signals for the on-board motor driver. A motor controller typically also has on-board sensors and on-board processing for monitoring and limiting current and changes in duty cycle, amongst other functionalities.

.. important::

  The :code:`pololu_smc_g2_constant.h` header file defines the various values specified in the board's documentation, e.g., register and command values.

.. important::

  The terminology "speed" is used throughout the documentation to refer to the duty cycle of the power being sent to the motor. Hence any use of "speed" in this class should be interpretted as "duty cycle".


.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2


Class definition
****************

.. cpp:class:: Pololu_SMC_G2

  This class implements the majority of the commands documented of the Pololu SMC G2 board, of which there are four versions that all use the same commands: 18v15, 18v25, 24v12, 24v19. The full listing of the boards's commands is found under:

  * `6.2.1 Binary command reference <https://www.pololu.com/docs/0J77/6.2.1>`_
  * `6.4 Controller variables <https://www.pololu.com/docs/0J77/6.4>`_


..
  Rather than set the global namespace like this:
  .. cpp:namespace:: Pololu_SMC_G2
  We choose to define the namespace explicitly for every item below.

  Hence empty the global namespace in case it is set elsewhere:

.. cpp:namespace:: nullptr


Member variables
****************

.. cpp:member:: private uint8_t Pololu_SMC_G2::m_i2c_address

  The I2C address of this class instance.

.. cpp:member:: private I2C_Driver * Pololu_SMC_G2::m_i2c_driver

  Pointer to the instance of the :cpp:class:`I2C_Driver` class that is manager the I2C bus to which this device is connected.


Class constructor
*****************

.. cpp:function:: Pololu_SMC_G2::Pololu_SMC_G2()

   Initialises the :cpp:member:`m_i2c_address` to the default of values of :code:`0x13` and leave the :cpp:member:`m_i2c_driver` unset.

.. cpp:function:: Pololu_SMC_G2::Pololu_SMC_G2(I2C_Driver * i2c_driver)

  Initialises the :cpp:member:`m_i2c_address` to the default of values of :code:`0x13` and sets the :cpp:member:`m_i2c_driver` to the pointer provided.

.. cpp:function:: Pololu_SMC_G2::Pololu_SMC_G2(I2C_Driver * i2c_driver, uint8_t address)

  Initialises the :cpp:member:`m_i2c_address` to the value provided (if it is a valid value) and sets the :cpp:member:`m_i2c_driver` to the pointer provided.


Getter functions
****************

.. cpp:function:: public uint8_t Pololu_SMC_G2::get_i2c_address()

  Returns a I2C device address stored in :cpp:member:`m_i2c_address`.


Setter functions
****************

.. cpp:function:: public bool Pololu_SMC_G2::set_i2c_address(uint8_t new_address)

  Sets the I2C device address in :cpp:member:`m_i2c_address` if a value between 0 and 127 (inclusive) is provided.

  | **Parameters:**
  |   **uint8_t new_address**
  |   Requested update of the I2C address.

  | **Returns: bool**
  |   Boolean flag indicating the status of the update
  |   :code:`true` for successful update of :cpp:member:`m_i2c_address`.
  |   :code:`false` address not updated due to requested value being out of the valid range.


Member functions
****************


Convenience function for initialisation
#######################################

.. cpp:function:: public bool Pololu_SMC_G2::initialise_with_limits(int new_current_limit_in_milliamps, int new_max_speed_limit, int new_max_accel_limit, int new_max_decel_limit, bool verbose)

  This function performs the following steps:

  * Sends the :cpp:func:`exit_safe_start` command.
  * Retrieves the error status using :cpp:func:`get_error_status` function.
  * Sleeps for 1 millisecond.
  * Gets the value of the input voltage using the :cpp:func:`get_input_voltage_in_volts` function.
  * Sleeps for 1 millisecond.
  * Sets the current limit to the specified value in milliamps using the :cpp:func:`set_current_limit_in_milliamps` function.
  * Sleeps for 1 millisecond.
  * Set the maximum allowed duty cycle for the motor commands using the :cpp:func:`set_motor_limit_max_duty_cycle` function.
  * Sleeps for 1 millisecond.
  * Set the maximum allowed change in duty cycle for the motor commands using the :cpp:func:`set_motor_limit_max_acceleration` and :cpp:func:`set_motor_limit_max_deceleration` functions. The acceleration and deceleration limits apply over the so-called "speed update period" (see `user manual Section 5.2 <https://www.pololu.com/docs/0J77/5.2>`_ for details), which has a default value of 1 millisecond.

  | **Parameters:**
  |   **int new_current_limit_in_milliamps**
  |   The current limit to set.

    | **int new_max_duty_cycle_limit**
    | The duty cycle limit to set.

    | **int new_max_accel_limit**
    | The duty cycle acceleration limit to set.

    | **int new_max_decel_limit**
    | The duty cycle deceleration limit to set.

    | **bool verbose**
    | Flag for whether to print out information during the initialisation steps (:code:`true`) or not (:code:`false`).

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success with all initialisation steps, :code:`false` otherwise.


Start and stop commands
#######################

.. cpp:function:: public bool Pololu_SMC_G2::exit_safe_start()

  Sends the so-called "exit safe-start" command to the board via I2C.

  As per the `user manual Section 6.2.1 <https://www.pololu.com/docs/0J77/6.2.1>`_ the exit safe-start command is described as:

    *If the input mode is Serial/USB, and you have not disabled safe-start protection, then this command is required before the motor can run. Specifically, this command must be issued when the controller is first powered up, after any reset, and after any error stops the motor. This command has no serial response.*

    *If you just want your motor to run whenever possible, you can transmit exit safe start and motor speed commands regularly. The motor speed commands are documented below. One potential problem with this approach is that if there is an error (e.g. the battery becomes disconnected) then the motor will start running immediately when the error has been resolved (e.g. the battery is reconnected).*

    *If you want to prevent your motor from starting up unexpectedly after the controller has recovered from an error, then you should only send an exit safe start command after either waiting for user input or issuing a warning to the user.*

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.

.. cpp:function:: public bool Pololu_SMC_G2::stop_motor()

  Sends the "stop motor" command to the board via I2C.

  As per the `user manual Section 6.2.1 <https://www.pololu.com/docs/0J77/6.2.1>`_ the stop motor command is described as:

    *This command sets the motor target speed (i.e., duty cycle) to zero and makes the controller susceptible to a safe-start violation error if safe start is enabled. Put another way, this command will stop the motor (configured deceleration limits will be respected) and not allow the motor to start again until the Safe-Start conditions required by the Input Mode are satisfied. This command has no serial response.*

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.


Convenience functions for commanding the motor duty cycle
#########################################################

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_target_duty_cycle_3200(int target_duty_cycle)

  Sets the target duty cycle for the motor via I2C. The units for this function are the maximum resolution possible for setting the duty cycle:

  * :code:`-3200` duty cycle is reverse at full speed.
  * :code:`0` motor stopped.
  * :code:`3200` duty cycle is forward at full speed.

  Note that acceleration and deceleration limits are respected when the motor controller implements the change from one target duty cycle to another target duty cycle.

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new target duty cycle to set, in "units" [-3200,3200] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_target_duty_cycle_percent(int target_duty_cycle)

  Sets the target duty cycle for the motor via I2C. The units for this function are percent with integer resolution.

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new target duty cycle to set, in "units" [-100,100] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_target_duty_cycle_percent(float target_duty_cycle)

  Sets the target duty cycle for the motor via I2C. The units for this function are percent with floating-point resolution.

  **Note:** this function convert the floating-point value of the duty cycle percentage to the maximum resolution range of [-3200,3200] and then rounds to the nearest integer.

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new target duty cycle to set, in "units" [-100,100] as a floating-point value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_target_duty_cycle_7bit(int target_duty_cycle)

  Sets the target duty cycle for the motor via I2C. The units for this function are 7-bit resolution for each direction, i.e., 8-bit (minus 1) resolution from full speed forward to full speed backwards.

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new target duty cycle to set, in "units" [-127,127] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.


Brake command
#############

.. cpp:function:: public bool Pololu_SMC_G2::motor_brake(int brake_amount)

  Sends the "brake motor" command via I2C.

  As per the `user manual Section 6.2.1 <https://www.pololu.com/docs/0J77/6.2.1>`_ the brake motor command is described as:

    *This command causes the motor to immediately brake or coast (configured deceleration limits are ignored). The brake amount byte can have a value from 0 to 32, with 0 resulting in coasting (the motor outputs are disabled) and any non-zero value resulting in braking (the motor outputs are driven low). Requesting a brake amount greater than 32 results in a serial format error. This command has no serial response.*

  | **Parameters:**
  |   **int brake_amount**
  |   The brake amount, in "units" [0,32] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.


Forward duty cycle commands
###########################

These are used by the convenience functions above to set the motor for forward (positive) commands.

.. cpp:function:: public bool Pololu_SMC_G2::motor_forward_3200(int target_duty_cycle)

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new forward target duty cycle to set, in "units" [0,3200] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.

.. cpp:function:: public bool Pololu_SMC_G2::motor_forward_percent(int target_duty_cycle)

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new forward target duty cycle to set, in "units" [0,100] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.

.. cpp:function:: public bool Pololu_SMC_G2::motor_forward_7bit(int target_duty_cycle)

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new forward target duty cycle to set, in "units" [0,127] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.


Reverse duty cycle commands
###########################

These are used by the convenience functions above to set the motor for reverse (negative) commands.

.. cpp:function:: public bool Pololu_SMC_G2::motor_reverse_3200(int target_duty_cycle)

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new reverse target duty cycle to set, in "units" [0,3200] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.

.. cpp:function:: public bool Pololu_SMC_G2::motor_reverse_percent(int target_duty_cycle)

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new reverse target duty cycle to set, in "units" [0,100] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.

.. cpp:function:: public bool Pololu_SMC_G2::motor_reverse_7bit(int target_duty_cycle)

  | **Parameters:**
  |   **int target_duty_cycle**
  |   The new reverse target duty cycle to set, in "units" [0,127] as an integer value.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for success, :code:`false` otherwise.




Workhorse functions for getting and setting via I2C
###################################################

.. cpp:function:: private bool Pololu_SMC_G2::get_variable(uint8_t variable_id, uint16_t * value)

  Retrieves the value of the requested variable via an I2C communication with the board.

  | **Parameters:**
  |   uint8_t variable_id
  |   The ID of the variable requested.

    | uint16_t * value
    | Pointer to where the retrieved value should be restored.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for successfully retrieved the value, :code:`false` otherwise.

.. cpp:function:: private bool Pololu_SMC_G2::set_motor_limit(uint8_t limit_id, uint16_t value, int * response_code)

  Set the value of the requested limit via an I2C communication with the board.

  | **Parameters:**
  |   uint8_t limit_id
  |   The ID of the limit to be set.

    | uint16_t value
    | Value to be set for that limit.

    | int * response_code
    | Pointer to where the response code should be stored.

  | **Returns: bool**
  |   Boolean flag indicating the status, :code:`true` for successfully set the value, :code:`false` otherwise.


List of functions for setting limits
####################################

**Set forward and reverse limits at the same time**

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_duty_cycle(int new_max_duty_cycle, int * response_code)

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_acceleration(int new_max_acceleration, int * response_code)

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_deceleration(int new_max_deceleration, int * response_code)

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_brake_duration(int new_max_brake_duration, int * response_code)


**Set forward limits**

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_duty_cycle_forward(int new_max_duty_cycle, int * response_code)

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_acceleration_forward(int new_max_acceleration, int * response_code)

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_deceleration_forward(int new_max_deceleration, int * response_code)

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_brake_duration_forward(int new_max_brake_duration, int * response_code)

**Set reverse limits**

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_duty_cycle_reverse(int new_max_duty_cycle, int * response_code)

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_acceleration_reverse(int new_max_acceleration, int * response_code)

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_deceleration_reverse(int new_max_deceleration, int * response_code)

.. cpp:function:: public bool Pololu_SMC_G2::set_motor_limit_max_brake_duration_reverse(int new_max_brake_duration, int * response_code)

**Set the current limit**

.. cpp:function:: public bool Pololu_SMC_G2::set_current_limit_in_internal_units(int new_current_limit)

.. cpp:function:: public bool Pololu_SMC_G2::set_current_limit_in_milliamps(int new_current_limit)


List of functions for getting variables
#######################################

**Get the firmware version**

.. cpp:function:: public bool Pololu_SMC_G2::get_firmware_version(uint16_t * product_id, uint8_t * firmware_major , uint8_t * firmware_minor)

**Get the status flag registers**

Refer to the `user guide Section 6.4 <https://www.pololu.com/docs/0J77/6.4>`_ for how to interpret the error flags.

.. cpp:function:: public bool Pololu_SMC_G2::get_error_status(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_error_occurred(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_serial_erros_occurred(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_limit_status(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_reset_flags(uint16_t * value)


**Get the RC channel values**

.. cpp:function:: public bool Pololu_SMC_G2::get_rc1_unlimited_raw_value(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_rc1_raw_value(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_rc1_scaled_value(int16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_rc2_unlimited_raw_value(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_rc2_raw_value(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_rc2_scaled_value(int16_t * value)

**Get the analog channel values***

.. cpp:function:: public bool Pololu_SMC_G2::get_an1_unlimited_raw_value(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_an1_raw_value(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_an1_scaled_value(int16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_an2_unlimited_raw_value(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_an2_raw_value(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_an2_scaled_value(int16_t * value)

**Get the command and duty cycle values**

.. cpp:function:: public bool Pololu_SMC_G2::get_target_duty_cycle_3200(int16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_duty_cycle_3200(int16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_brake_amount(uint16_t * value)

**Get some diagnostic values (voltage, temperture, ...)**

.. cpp:function:: public bool Pololu_SMC_G2::get_input_voltage_in_volts(float * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_temperature_a(float * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_temperature_b(float * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_rc_period_in_seconds(float * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_baud_rate_register_in_bps(float * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_up_time_low(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_up_time_high(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_up_time_in_seconds(float * value)


**Get motor duty cycle limits (forward)**

.. cpp:function:: public bool Pololu_SMC_G2::get_max_duty_cycle_forward(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_max_acceleration_forward(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_max_deceleration_forward(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_brake_duration_forward(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_brake_duration_forward_in_seconds(float * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_starting_duty_cycle_forward(uint16_t * value)


**Get motor duty cycle limits (reverse)**

.. cpp:function:: public bool Pololu_SMC_G2::get_max_duty_cycle_reverse(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_max_acceleration_reverse(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_max_deceleration_reverse(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_brake_duration_reverse(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_brake_duration_reverse_in_seconds(float * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_starting_duty_cycle_reverse(uint16_t * value)


**Get the current and current limit values**

.. cpp:function:: public bool Pololu_SMC_G2::get_current_limit(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_raw_current(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_current_in_milliamps(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_current_limiting_consecutive_count(uint16_t * value)

.. cpp:function:: public bool Pololu_SMC_G2::get_current_limiting_occurrence_count(uint16_t * value)

