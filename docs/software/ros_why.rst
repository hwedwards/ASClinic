.. _ros-why:

Why use ROS (for robotics)?
===========================

The `ROS wiki introduction page <https://wiki.ros.org/ROS/Introduction#Goals>`_ under the heading "Goals" provides some themes for answering this question. Using the same heading but with different descriptions, the goals of ROS are list below.

.. admonition:: Some probing question to keep in mind

  * Are these the goals and functionality you would anyway create if you started from scratch?
  * Is ROS a blessing or a necessary evil?


* **Thin:** There is a large variety of hardware out there, which often comes with its own software for interfacing with that hardware. The nature of getting a robot working in the real-world is that the hardware and software use are large variety of techniques to achieve reliable operation. The "thin" goal of ROS could be stated as a goal that ROS is flexible enough to integrate with whatever hardware and software that you have chosen to be appropriate for your robotics project.
  * This goal of ROS makes it worth using because it (most likely) will not cause you be paint yourself into a corner.

* **ROS-agnostic libraries:** As described in answer to "what is ROS", it is a middleware that facilitates easy management and message passing between processes and the programming interface for this is obviously specific to ROS. Beyond that, ROS libraries are predominantly developed in a way that is agnostic to way ROS function under-the-hood, and instead focus on providing a clean functional interface.
  * This goal of ROS makes it worth using because it prefers a functional description that describes what the robot is doing and its algorithmic architecture, as opposed to a functional description bogged-down and confounded by ROS-specific syntax and architecture.

* **Language independence:** Each programming language has its strengths and weaknesses. When choosing a programming language for your robotics project, ideally you can freely choose the language that yields the best outcome, be that the best language for computation speed (possibly C++), or the best language for usability (possibly Python). Moreover, different aspects of the robotics project will (almost surely) have different requirements, for example: speed of computer vision computation versus usability of providing high-level mission descriptions. Currently ROS supports C++, Python, and Lisp, and has experimental support for Java and Lua. The feature of ROS to send messages that are agnostic to the programming language used for sending and receiving the messages is key enabler for a mixed use of programming languages.
  * This goal of ROS makes it worth using because your choice of middleware should not restrict your ability to select the most appropriate language for each aspect of your robotics project.

* **Easy testing:** Occasionally, or perhaps frequently, during the development of your robotics project hardware will break, for example: mechanical joints, sensors, actuators. As the robot gets more complex, parts get more expensive, and demands for reliability increase, such breakages become more and more undesirable. A unit testing and integration framework helps to prevent avoidable breakages.
  * This goal of ROS makes it worth using because as your abilities with ROS increase, so too does your ability to guarantee the performance and behaviour of your software.

* **Scaling:** What may start out as two wheels, two motors, and a camera, can quickly become a heterogeneous multi-agent system relying on edge and cloud computing to carry out tasks in a distributed fashion. The scaling features of ROS readily allows nodes to be running and communicating across multiple computers on the same network.
  * This goal of ROS makes it worth using because even a small robotics project can benefit from distributing the nodes across multiple computers.


.. important::
  Each time you feel that you have gained new skills and mastery with ROS, take some time to write down another alternative answer to the question: why use ROS?