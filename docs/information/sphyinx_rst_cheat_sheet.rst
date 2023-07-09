.. _sphyinx-rst-cheat-sheet:

SPHYINX RST CHEAT SHEET
=======================

A few example resources for cheat sheets:

* `The Sublime and Sphyinx Guide <https://sublime-and-sphinx-guide.readthedocs.io/en/latest/code_blocks.html>`_

* `reStructuredTest and Sphinx reference guide on documatt <https://restructuredtext.documatt.com/index.html>`_


.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2


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

Nested lists must be indent to the same level as the text starts. This is easy to miss when using an enumerated list because it requires an indent of 3 spaces instead of the usual 2 space indent.

The following does NOT properly produced an indented sub-list:

.. code-block::

  #. Main list item.

    #. This item is NOT indented enough.

The following DOES produced an indented sub-list:

.. code-block::

  #. Main list item.

     #. This item is indented enough, i.e., by 3 spaces.


Contents of an individual .rst file
***********************************

The :code:`contents` directive inserts a hyperlinked list of all the heading in that .rst file. The directive with its option:

.. code-block::

  .. contents:: Contents of this page
     :local:
     :backlinks: none
     :depth: 2

For which the results can be seen at the top of this page. The options are well explained on `this documatt page
<https://restructuredtext.documatt.com/element/contents.html>`_, and summarised as:

  * :code:`:local:` exclude the top-level title of this page from the contents.
  * :code:`:backlinks: none`, remove hyperlinks from the section heading back to the contents.
  * :code:`:depth: 2`, specifies the depth to display in the contents.

.. note::

  The :code:`contents` directive must be used before the first sub-heading of the page.


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

.. warning::

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




