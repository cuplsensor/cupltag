Operating Modes
-----------------

Primary
~~~~~~~~

The software progresses into the loop at the bottom of the :ref:`StateChart`:
* Programming mode is not entered because the nPRG pin is deasserted.
* Tag configuration is present according to :cpp:func:`init_configcheck`.
* No evidence of repeated resets has been found by :cpp:func:`init_errorcheck`.

In normal operation the tag is updated periodically with calls to cuplCodec. To conserve power the
MSP430 Real Time Clock (RTC) is set to generate interrupts at one minute intervals. The RTC
is clocked by the 32.768 kHz watch crystal. A majority of time is spent
waiting in standby (LPM3) for the next RTC interrupt. The VMEM domain that includes the NT3H2211 NFC tag
and the HDC2010 is powered off during this time.
Therefore cuplTag draws little more than 1.43uA; equal to the MSP430 consumption in LPM3 (`MSP430Datasheet`_).

Every Minute
*************



At the Sample Interval (in minutes)
**************************************


.. _MSP430Datasheet: https://www.ti.com/document-viewer/MSP430FR2155/datasheet/operating-modes-slasec45810#SLASEC45810