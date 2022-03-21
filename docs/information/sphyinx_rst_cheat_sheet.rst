.. _sphyinx-rst-cheat-sheet:

SPHYINX RST CHEAT SHEET
=======================

Section headings
****************

A heading in define by underlining the titile with a puncuation character that is at least as long as the title. The typical punction marks to use are :code:`= * # - ^ "` and their usage should be consitent within a project.

For this wiki, we use the following:
* :code:`=` for the top-level title, e.g., the title of this page.
* :code:`*` for section headings, e.g., the "Section headings" heading of this section (that is a mouthful!).
* :code:`#` for subsection headings.
* :code:`-` for subsubsection headings.


Lists
*****

Nested lists must be indent to the same level as the text starts.


Admonition directives
*********************

Admonition directives are those that provide a coloured box to highlight a particular type of information.


With custom title
#################

The :code:`admonition` directive allows you to give any title to the admonition block. The syntax is:

.. code-block::

  .. admonition:: Custom title for this admonition
    
    Contents go here for this admonition with a custom title.

.. admonition:: Custom title for this admonition
  
  Contents go here for this admonition with a custom title.

With pre-specified titles
#########################

The following are perhaps the two most common admonition directives with a specified title.

.. note::

  This is a note admonition, created using the syntax :code:`.. note:: Contents of the note goes here.`

.. warning:: I'm warning.

  This is a warning admonition, created using the syntax :code:`.. warning:: Contents of the warning goes here.`

The other eight admonitions that are avaialble with a specified title are:

.. attention:: This is a attention admonition, created using the syntax :code:`.. attention:: Contents of goes here.`

.. caution:: This is a caution admonition, created using the syntax :code:`.. caution:: Contents of goes here.`

.. danger:: This is a danger admonition, created using the syntax :code:`.. danger:: Contents of goes here.`

.. error:: This is a error admonition, created using the syntax :code:`.. error:: Contents of goes here.`

.. hint:: This is a hint admonition, created using the syntax :code:`.. hint:: Contents of goes here.`

.. important:: This is a important admonition, created using the syntax :code:`.. important:: Contents of goes here.`

.. tip:: This is a tip admonition, created using the syntax :code:`.. tip:: Contents of goes here.`

.. seealso:: This is a seealso admonition, created using the syntax :code:`.. seealso:: Contents of goes here.`




