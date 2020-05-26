Getting Started
----------------

Prerequisites
~~~~~~~~~~~~~~~
* MSP-FET debugger (`TI <https://www.ti.com/tool/MSP-FET>`_).
* GitHub Desktop (`download <https://desktop.github.com/>`_) or your choice of Git software.

Fork and Clone cupl Tag
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. image:: fork-and-clone.png
   :width: 400

Visit the cupl Tag repository on (`GitHub <https://github.com/cuplsensor/cupltag>`_). Click the
fork button in the top right.

Clone the forked repository to your computer by clicking the green Clone or Download button. See
`<https://guides.github.com/activities/forking/>`_ for more details.

Open the Code Composer Project
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#. `Download <https://software-dl.ti.com/ccs/esd/documents/ccs_downloads.html>`_ and install Code Composer Studio 10 (CCS).
#. Open CCS. Launch the workspace of your choice.
#. Click File -> Open Projects from File System...
#. In Import Source, select the folder containing the clone of cupl Tag.

    .. image:: ccsimport.png
       :width: 400

#. Ensure that cupltag\\firmware is **checked**. Every other folder should be unchecked.
#. Click Finish.
#. The project is now open.

    .. image:: ccsopen.png
       :width: 400

Add Reference to cupl Codec
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The cupl Tag firmware depends on C files from cupl Codec.

#. Fork the cupl Codec repository on (`GitHub <https://github.com/cuplsensor/cuplcodec>`_).
#. Clone this into a folder on your computer.
#. In CCS, right click the cuplTag_firmware project.
#. Select Properties from the context menu.
#. In the Properties window, expand Resource on the left hand panel and select Linked Resources.

    .. image:: ccslinkedresources.png
       :width: 400

#. Double click the CUPLCODEC path variable. The Edit Variable window will appear.

    .. image:: ccseditpathvar.png
       :width: 400

#. Click the Folder... button. Select the Codec clone folder from step 2.
#. Click Apply and Close.
#. The cuplcodec_encoder project folder will now contain references to files inside cupl Codec.

    .. image:: ccsreferenceadded.png
       :width: 400