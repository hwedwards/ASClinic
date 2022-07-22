.. _how-to-use-the-wiki:

HOW TO USE THE WIKI
===================

Summary
*******

Almost everything contained in this wiki can be found elsewhere, for example: text books; online tutorials and documentation; datasheets; stack-overflow pages; forums; blog posts; wikipedia. One key reason for bringing this information together in this wiki is that it provides a safety net for achieving a baseline operation of your robot.

In order to gain the most transformative experience out of your robotics project, we recommend that you use this wiki with the frame of mind to bring together the information yourself. By this, we mean that when you are working on a particular part of of the project, (for example: robot operating system (ROS)), then you:

- First: study other available resources related to that part and write/derive/code/design your own understanding and solution for that part.
- Second: review what is contained in this wiki to as point of comparison with what you developed.

To facilitiate and guide the first step described above, parts of this wiki begin, where appropriate, by posing the contents of that part as a sequence of questions that you should attempt answer yourself. In some cases you may be able to answer the questions based on your prior knowledge, for example, to dervive a model of a differential drive robot based on modelling techniques you learned in a prior subject. In other cases, you may need to invest time into learning from other resources to gain the knowledge, for example, how to use and work with git versioning control.

The following is an example of such questions for the ROS part of this wiki, for which you might choose to use the offical ROS wiki and video tutorials to formulate your own answers to these questions.

.. admonition:: Contents of this part posed as questions

  #. :ref:`ros-what` Explain at a high-level.
  #. :ref:`ros-why` Explain why the features of ROS are considered useful for developing a robot.
  #. Explain each of :ref:`ros-key-elements`, i.e., explain each of the following:

     - Nodes
     - Topics
     - Messages
     - Publishing and subscribing
     - Services

     Additionally explain how these elements are used together, drawing an example to support your explanation.

.. hint::

  Also compare the answers that you formulate with the answers formulated by your teammates, peers, and others is your cohort.


Reasons for using the wiki in this way
**************************************

Inherent in the defintion of a robotics project, or any project for that matter, is that there is NOT a single source that provides you with a fully detailed step-by-step guide to completing the whole project. And if you do find such a guide, then you can consider adjusting the goals of your project.

The very nature of this wiki means that it looks a lot like a step-by-step guide. But it is a guide for some parts of the project; it is not a guide for bringing all those parts together into a working whole that achieves the specific goals of your project.

..
  Completing your robotics project requires you to understand what software, hardware, and information is already out there so that you can appropritely leverage it to achieve the specific goals of your project.




Too Long; Didn't Read (TL;DR)
*******************************

There is certainly more text books, stack-overflow pages, blog posts, wikis, instructions, datasheets, online videos, etc., than there is time available to read and watch all of them. So it is part of sound engineering judgement to decide how and where to invest your time with reading and comprehending what is out there in order to apply it to create, analyse, and evaluate your robotics project.

Two extreme scenarios are:

* Treat all infromation as TL;DR. What this often results in is haphazardly bolting hardware and software together until the status is: "it works... occassionaly... and we don't really know what is going on".
* Read every related datasheet and internet post in full detail. What this often results in is running out of time for producing a functioning robot.

.. important::

  Most likely your project will proceed somewhere between these two extremes as you and your team should actively make decisions about the approprite level of detail when researching and pursing the parts of the project.


Too Tedious; Didn't Bother (TT;DB)
**********************************

Similar to TL;DR above, there is certainly more pages in your note book than it would require for an ad hoc approach to eventually function, let alone to be robust and reliable. And most engineering designs involve more parameters than a brute force approach can address within a lifetime of computation. So it is part of sound engineering judgement to decide how and where to invest your time with applying your engineering knowledge, engineering skills, engineering judgement, and engineering experiences to each part of your robotics project.

Two extreme scenarios are:

* Treat all and all engineering design and analysis steps as "too tedious; didn't bother". What this often results in is randomly tuning parameters and designs until the status is: "it works... occassionaly... and we don't really know what is going on".
* Derive every detail for every part of the project from the fundamental axioms of mathematics and principles of physics. What this often results in is running out of time for producing a functioning robot.

.. important::

  It is surprisingly easy to fall unwittingly into a TT;DB mode of operation, especially with time pressures to get your project finished and other activities drawing on your attention. For example, this could be verbalised as: "I tried every combination of plus and minus and none of them works". Be sure to occasionally step back, assess your mode of operation, and if you discover you are in the TT;DB mode, then quietly admit it to yourself and shift gears into a new mode.


What this wiki is NOT
*********************
It is not feasible to guide you through all steps for all part because, by definition, your robotics project is open-ended and has many plausible solution paths. Hence an attempt at such a guide would quickly become convoluted and unwieldy. And you already have internet search engines at your finger tips for exploring the vast number of posts discussing almost every challenge and error message you will face.

Additionally, it is not sensible to guide you through all steps of a particular solution path because a hallmark of project-based learning is that you learn through exploratory experience. Although following a step-by-step guide is itself an experience, in general there will not be a step-by-step start-to-finish guide for projects that you will undertake during your professional engineering career.

What this wiki is
*****************
This guide is a composition of many short step-by-step guides for a variety of aspects that are applicable to many different robotics projects. The goal is to get you familiar and competent with the various sub-systems of the robot platform so that you can then head-off on the exploratory trail of the robotics project you want to pursue.

How this wiki suits you
***********************
Every engineer, engineering student, teacher, and hobbyist comes to the table with a unique set of skills and experiences. For instance, an individual may be an expert in one area and a novice in another area, for example having engineered lots of custom projects with Arduino but never installed or used a Linux distribution. Hence an expert will be bored by a step-by-step guide aimed at a novice, and a novice will be derailed by a step-by-step guide aimed at an expert.

Hence the onus is on you to self-asses whether you need to follow every step of a guide, or whether you can skip ahead to the steps that are specific to the hardware and software involved.


Be realistic
************
Everything on this wiki is, by definition, incomplete because:

* There are so many different resources out there on similar topics, and so many different types of {users, makers, hackers, student engineers} that it is not clear which one or more resources will guide you mostly clearly.
* Software versions and hardware design are changing at a pace faster than this wiki is updated.



..
  Topics covered as a Table of Contents:
  The following list gives an overview of the topics covered by the guides, and hence it implies one possible order for working through the guides. However, there is not a prescribed order in which you must work through the guides, which this is highlighted by the fact that the guides are not even sub-pages of this part of the wiki. The ordered implied by the following list is considered to be most suitable for a person with very little prior experience working with the hardware and software involved. For a person with more experience, you can also work through the following list in order and skip over parts where they already feel competent. For a person with a very specific situation to address, the following list may assist with navigating to the guide that is most relevant to your situation.


..
  Purpose statement
  In order to make your robot project a reality, you will need to draw upon many different skills. One broad categorization of the required skills is:
  > System modelling based on sound engineering theory and assumptions that are appropriate.
  > Algorithm design based on sound engineering methodologies.
  > Implementation in hardware and software.
  > Experiment design, data analysis, problem solving.
  > Project management, design iteration, decision making, and integration/interfacing across the system components.

