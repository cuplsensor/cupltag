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
and the HDC2021 is powered off during this time.
Therefore cuplTag draws little more than 1.43uA; equal to the MSP430 consumption in LPM3 (`MSP430Datasheet`_).

Every Minute
*************

#. cuplTag wakes up from standby (LPM3).
#. Minute counter is incremented.
#. The VMEM domain is powered on.
#. A configuration file check is made. A reset occurs if one is found.
#. A call is made to enc_setelapsed in cuplCodec. The minuteselapsed field (CODEC_FEAT_26) of the cuplCodec URL
   is updated.
#. The VMEM domain is powered off.
#. cuplTag returns to LPM3.


At the Sample Interval (in minutes)
**************************************

#. cuplTag wakes up from standby (LPM3).
#. The minute counter is reset to 0.
#. The VMEM domain is powered on.
#. A configuration file check is made. A reset occurs if one is found.
#. A sample is requested from the humidity sensor.
#. The MSP430 waits in LPM3 until the DRDY line of this sensor is asserted.
#. The sample is read from the humidity sensor.
#. A call is made to enc_pushsample in cuplCodec. The sample is written to the circular
   buffer inside the cuplCodec URL. The minuteselapsed field is reset to 0.
#. If the circular buffer has wrapped around to the start, then a call is made to :cpp:func:`nvparams_cresetsperloop`.
#. The VMEM domain is powered off.
#. cuplTag returns to LPM3.



Configuration file check
Block 1 of the NT3H2211 is read via I2C. If it contains a text record, then it is assumed
that a configuration file has been written. cuplTag resets to read the configuration file.

.. _MSP430Datasheet: https://www.ti.com/document-viewer/MSP430FR2155/datasheet/operating-modes-slasec45810#SLASEC45810