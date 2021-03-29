.. _voltaic-v88:

Voltaic V88 Power Bank
======================

The Voltaic V88 power bank can be viewed on the manufacturers website `here <https://voltaicsystems.com/v88/>`_, and the manual can be viewed online `here <https://voltaicsystems.com/content/Voltaic%20Systems%20V88%20Instruction%20Booklet.pdf>`_.

The following provides the most relevant details extracted directly from the manuel. Then the instructions for using the Voltaic V88 to power a robot device are given.

.. _voltaic-v88-on-off:

On/Off Switch
*************

To turn off all outputs, double tap the Power Button. Press again to turn on.

.. _voltaic-v88-change-voltage:

To Change Voltage
*****************

Remove DC Out Cable. Hold the Power Button until the Output Voltage flashes (3 seconds), then release. Press again to cycle through voltage levels. Reconnect DC Out Cable to power device.

.. _voltaic-v88-always-on:

Always On Mode
**************

Recommended for IoT applications. Prevents battery from turning off automatically. Hold the Power Button until the LED screen reads “Always On Mode” (~5 seconds). For more: `<www.voltaicsystems.com/always-on/>`_

Auto Off Mode
*************

For everyday use and long term storage use Auto Off mode. To switch back to this mode hold Power Button until “Always On Mode” disappears from LED screen (~ 5 seconds).

Tips
****

If the V88 turns off unexpectedly, connect to a power source to reset.

.. _voltaic-v88-charge-from-ac:

Charge From AC Power
********************

Connect the 20V AC Charger to the **DC Input Port** on the V88. The Battery Level Indicator will flash in sequence.

.. warning:: Charge V88 fully every 5 months to preserve battery life. Store between -20 to 35 degrees C (-4 to 95 degrees F). Charge between 0 to 45 degrees C (32 to 113 degrees F). Do not expose the battery to high heat or direct flame. Dispose of properly. Do not throw in trash or incinerate. Do not get battery wet, or store in high humidity, or otherwise short circuit. Do not disassemble or tamper with.

Specifications
**************

* **Battery:** Li-Polymer, 24,000mAh, 88.8 Watt-hours
* **Output:** 12V/6A, 16V/5A, 19V/4.5A, 24V/3.5A
* **USB QC:** 5V/3A, 3.6V-12V, 18W Max.
* **USB-C PD:** 5V/3A-20V/2.2A
* **Input:** 18-25V/2A

Power a Robot
*************

1. :ref:`Turn on <voltaic-v88-on-off>` the Voltaic V88 power bank.
2. If the Voltaic V88 has two or fewer bars, then :ref:`charge it <voltaic-v88-charge-from-ac>` before proceeding.
3. :ref:`Select the voltage <voltaic-v88-change-voltage>` necessary for your robot.
4. :ref:`Turn off <voltaic-v88-on-off>` the Voltaic V88 power bank.
5. Connect the power jack of your robot to the **DC Output Port** of the Voltaic V88.
6. Press the power button on the Voltaic V88 to begin powering your robot.
7. Put the Voltaic V88 in :ref:`Always On Mode <voltaic-v88-always-on>` to prevent an unexpected shutdown during periods of low current draw.

.. warning:: Do NOT simultaneously charge the Voltaic V88 power bank and use it to power your robot.

.. warning:: The **DC Input Port** and **DC Output Port** of the Voltaic V88 are identical. Double check the you are using the input port for charging the power bank, and using the output port for powering your robot
