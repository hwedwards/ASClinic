.. _modelling-differential-drive-robot:

DIFFERENTIAL-DRIVE ROBOT
========================


Schematic and definitions
*************************

The most important starting point is to have a schematic that defines the notation and conventions we use to model the robot, in this case a differential-drive robot.

.. figure:: ../images/tikz-fig-dd-robot-definitions.pdf
  :width: 100%
  :scale: 90%
  :align: center
  :alt: Image failed to load

  Schematic of a differential-drive robot showing the definitions of the coordinate axes, states, and key physical dimensions.

The notation and quantities defined in the schematic are as follows:

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



.. _modelling-differential-drive-robot-kinematic-model:

Kinematic model
***************

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



