.. _modelling-wheel-odometry:

WHEEL ODOMETRY
==============

The wheel odometry presented here is for estimating the pose (i.e., position and orientation) based on measurements of the rotation of the left and right wheels on a non-steering axel. The measurement of each wheel must provide both direction of rotation and amount of rotation at known time intervals.

It does not matter whether the wheels are actively driven by a motor or passively rotating as the vehicle moves. Hence wheel odometry is applicable for differential drive robots and for the non-steering wheels of a car.

.. note::

  The measurements of wheel rotation are proprioceptive, which is to say that the measurements are internal to the robot itself. This immediately informs us of two key properties for the wheel odometry approach derived on this page:

  * The wheel odometry pose-estimate is relative to a given starting point.
  * The uncertainty of a wheel odometry pose-estimate continually increases.


Schematic and definitions
*************************

The most important starting point is to have a schematic that defines the notation and conventions we use to model the robot, in this case a differential-drive robot.

.. figure:: ../images/tikz-fig-dd-wheel-odometry-definitions.pdf
  :width: 100%
  :scale: 90%
  :align: center
  :alt: Image failed to load

  Schematic of a differential-drive robot showing the definitions of the coordinate axes, states, and key physical dimensions.

The notation and quantities defined in the schematic are as follows, and agree with the notation introduces for the :ref:`differential-drive robot model <modelling-differential-drive-robot>`:

.. list-table:: **COORDINATE AXES**
  :width: 100
  :widths: 20 80
  :class: longtable
  :header-rows: 0
  :stub-columns: 0
  :align: center

  * - :math:`W`
    - Symbol for the **world** frame

  * - :math:`(X^{(W)},Y^{(W)})`
    - Coordinate axes of the world frame

  * - :math:`B`
    - Symbol for the **body** frame

  * - :math:`(X^{(B)},Y^{(B)})`
    - Coordinate axes of the body frame




.. note::

  Some common alternatives to this coordinate axes notation and convention:

    * The "world" frame is often called the "inertial" frame, and hence :math:`(x^{(I)},y^{(I)})` is often used.
    * The "body" frame is often called the "robot" frame, and hence :math:`(x^{(R)},y^{(R)})` is often used.


.. list-table:: **STATE DEFINITIONS**
  :width: 100
  :widths: 20 80
  :class: longtable
  :header-rows: 0
  :stub-columns: 0
  :align: center

  * - :math:`p_x`, :math:`p_y`
    - Position of the body frame origin relative to the world frame origin, expressed in the coordinates of the world frame

  * - :math:`\phi`
    - Heading angle of the body frame relative to the world frame, i.e., the orientation of the robot, i.e., angle from the positive :math:`x_W` axes to the positive :math:`x_B` axes using a right-hand rule convention for the sign.

  * - :math:`\vec{p} = \begin{bmatrix}p_x \\ p_y \\ \phi\end{bmatrix}`
    - Vector collecting together the full **pose** of the robot, i.e., the pose of a rigid body in a 2D-space is fully described by its x-y position and its heading angle.



.. list-table:: **PHYSICAL DIMENSIONS**
  :width: 100
  :widths: 20 80
  :class: longtable
  :header-rows: 0
  :stub-columns: 0
  :align: center

  * - :math:`b`
    - The length of the wheel base, measured in meters

  * - :math:`r_l`, :math:`r_r`
    - Radius of the left and right wheels respectively, measured in meters.



Assumptions
***********

* No slip of the wheels
* The measured changes in wheel rotation, i.e., :math:`\Delta \theta_l` and :math:`\Delta \theta_r`, occurs at constant wheel velocity over the time duration :math:`\Delta t` of the measurement.

Pose estimate update equation
*****************************

Re-iterating that the measured changes in wheel rotation, i.e., :math:`\Delta \theta_l` and :math:`\Delta \theta_r`, are the inputs for computing a pose, we can use the same transformation as derived for the differential-drive kinematic model to convert this to:

* Change in forward position in the body frame, which we denote :math:`\Delta s` and compute as:

  .. math::

    \Delta s \,=\, \frac{r_l}{2} \, \Delta \theta_l \,+\, \frac{r_r}{2} \, \Delta \theta_r

* Change in rotation of the body frame, which we denote :math:`\Delta \phi` and compute as:

  .. math::

    \Delta \phi \,=\, \frac{r_l}{2b} \, \Delta \theta_l \,+\, \frac{r_r}{2b} \, \Delta \theta_r

These 

.. math::

  \begin{align}
  \begin{bmatrix}\hat{p}_{x,t+1} \\ \hat{p}_{y,t+1} \\ \hat{\phi}_{t+1}\end{bmatrix}
  \,=&\,
  \begin{bmatrix}\hat{p}_{x,t+1} \\ \hat{p}_{y,t+1} \\ \hat{\phi}_{t+1}\end{bmatrix}
  \,+\,
  \begin{bmatrix}\Delta p_{x,t} \\ \Delta p_{y,t+1} \\ \Delta \phi_{t}\end{bmatrix}
  \\
  =&\,
  \begin{bmatrix}\hat{p}_{x,t+1} \\ \hat{p}_{y,t+1} \\ \hat{\phi}_{t+1}\end{bmatrix}
  \,+\,
  \begin{bmatrix}
    \Delta s_{t} \, (\cos(\hat{\phi}_{t} + \frac{1}{2} \Delta \phi_{t+1}))
    \\
    \Delta s_{t} \, (\sin(\hat{\phi}_{t} + \frac{1}{2} \Delta \phi_{t+1}))
    \\
    \Delta \phi_{t}
  \end{bmatrix}
  \end{align}

This update equation for the wheel odometry estimate is equivalent to performing the following at each time step:

* Linearize the :ref:`differential drive kinematic model <modelling-differential-drive-robot-kinematic-model>` around the heading angle :math:`(\hat{\phi}_{t} + \frac{1}{2} \Delta \phi_{t+1})`.
* Assume constant velocity wheel rotations over the time interval that :math:`\Delta \theta_l` and :math:`\Delta \theta_r` were measured.
* Integrate the linearized equations forward for the measurement time interval :math:`Delta t`.


.. math::

   \dot{\vec{p}}
   =
   \begin{bmatrix}\dot{x}_p \\ \dot{y}_p \\ \dot{\phi}\end{bmatrix}
   =
   \begin{bmatrix}\cos(\phi) & 0 \\ \sin(\phi) & 0 \\ 0 & 1\end{bmatrix}
   \begin{bmatrix}v \\ \omega\end{bmatrix}


.. math::

   \begin{bmatrix}v \\ \omega\end{bmatrix}
   =
   \begin{bmatrix}\frac{r}{2} & \frac{r}{2} \\ \frac{-r}{2b} & \frac{r}{2b}\end{bmatrix}
   \begin{bmatrix}\dot{\theta}_l \\ \dot{\theta}_r\end{bmatrix}


.. math::

   \begin{bmatrix}\dot{x}_p \\ \dot{y}_p \\ \dot{\phi}\end{bmatrix}
   =
   \begin{bmatrix}\frac{r}{2}\cos(\phi) & \frac{r}{2}\cos(\phi) \\ \frac{r}{2}\sin(\phi) & \frac{r}{2}\sin(\phi) \\ \frac{-r}{2b} & \frac{r}{2b}\end{bmatrix}
   \,
   \begin{bmatrix}\dot{\theta}_l \\ \dot{\theta}_r\end{bmatrix}




Parameters for a specific robot
*******************************



|

----

.. image:: https://i.creativecommons.org/l/by/4.0/88x31.png
  :alt: Creative Commons License
  :align: left
  :target: http://creativecommons.org/licenses/by/4.0/

| Paul N. Beuchat, 2023
| This page is licensed under a `Creative Commons Attribution 4.0 International License <http://creativecommons.org/licenses/by/4.0/>`_.

----

|
