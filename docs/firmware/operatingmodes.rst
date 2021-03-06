Operating Modes
-----------------

.. _NormalMode:

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
#. A call is made to :cpp:func:`confignfc_check`. This checks if an NDEF text record
   (assumed to contain configuration data) is present on the tag. If so, a reset occurs.
#. A call is made to :cpp:func:`enc_setelapsed` in cuplCodec. The minuteselapsed field (CODEC_FEAT_26) of the cuplCodec URL
   is updated.
#. The VMEM domain is powered off.
#. cuplTag returns to LPM3.


At the Sample Interval (in minutes)
**************************************

#. cuplTag wakes up from standby (LPM3).
#. Minute counter is reset to 0.
#. The VMEM domain is powered on.
#. A call is made to :cpp:func:`confignfc_check`. This checks if an NDEF text record
   (assumed to contain configuration data) is present on the tag. If so, a reset occurs.
#. A sample is requested from the humidity sensor with :cpp:func:`hdc2010_startconv`.
#. The MSP430 waits in LPM3 until the DRDY line of this sensor is asserted.
#. The sample is read from the humidity sensor with :cpp:func:`hdc2010_read_temp`.
#. A call is made to :cpp:func:`enc_pushsample` in cuplCodec. The sample is written to the circular
   buffer inside the cuplCodec URL. The minuteselapsed field is reset to 0.
#. If the circular buffer has wrapped around to the start, then a call is made to :cpp:func:`nvparams_cresetsperloop`.
#. The VMEM domain is powered off.
#. cuplTag returns to LPM3.


Configuration file check
Block 1 of the NT3H2211 is read via I2C. If it contains a text record, then it is assumed
that a configuration file has been written. cuplTag resets to read the configuration file.

.. _ProgMode:

Secondary
~~~~~~~~~~~

The secondary operating mode is programming mode. The state :cpp:func:`init_progmode` is entered when the nPRG pin
is low after reset. A reset is triggered at power on or by a low pulse on the nRESET pin (see :ref:`HT04Pinout`).
The only way to leave programming mode is to trigger a reset. This is done either via the aforementioned means
or by sending the soft-reset command.

The serial port is active in this state and not in any other to save power. Connect with these settings:

+--------------+-------+
| Setting      | Value |
+==============+=======+
| Baudrate     | 9600  |
+--------------+-------+
| Parity       | None  |
+--------------+-------+
| Stop bit     | 1     |
+--------------+-------+
| Flow control | Off   |
+--------------+-------+

A simple command and response scheme is used. Basic commands have 3 characters:

+-----------+-----------------+---------------------------------------------+
| Character | Description     | Note                                        |
+===========+=================+=============================================+
|     <     | Start character |                                             |
+-----------+-----------------+---------------------------------------------+
|     z     | Command ID      | Any character in the range a-z, A-Z and 0-9 |
+-----------+-----------------+---------------------------------------------+
|     >     | End character   |                                             |
+-----------+-----------------+---------------------------------------------+

Configuration string commands add a parameter string:

+-----------+------------------+----------------------------------------+
| Character | Description      | Note                                   |
+===========+==================+========================================+
|     <     | Start character  |                                        |
+-----------+------------------+----------------------------------------+
|     b     | Command ID       | Any character in [a-z, A-Z, 0-9]       |
+-----------+------------------+----------------------------------------+
|     :     | Parameter prefix |                                        |
+-----------+------------------+----------------------------------------+
| ABcd1234  | Parameter string | Up to 64 characters in [a-z, A-Z, 0-9] |
+-----------+------------------+----------------------------------------+
|     >     | End character    |                                        |
+-----------+------------------+----------------------------------------+

Responses take a similar format to commands, starting with a '<' character and ending with a '>'.

A human-readable ASCII format was chosen because very little data is transacted.
It is useful to be able to send and receive commands through the terminal window without having to encode
and decode packets.

Basic Commands
***************

+---------+-----------+------------------------+--------------+---------------------------------------+
| Command | Name      | Response               | Example      | Description                           |
+=========+===========+========================+==============+=======================================+
| <x>     | Version   | <HWVER_FWVER_CODECVER> | <HT04_F2_C1> | Hardware, firmware and codec versions |
+---------+-----------+------------------------+--------------+---------------------------------------+
| <y>     | EnterBL   | None                   |              | Enter the MSP430 UART bootloader      |
+---------+-----------+------------------------+--------------+---------------------------------------+
| <z>     | SoftReset | None                   |              | Reset the MSP430                      |
+---------+-----------+------------------------+--------------+---------------------------------------+

Error Response
****************

The cuplTag firmware responds with '<e>' if it has failed to parse a command.

Configuration Commands
***********************

See configuration strings.


The :ref:`StateChart` shows a theoretical transition into an error state. This can only occur if the UART
state table is incomplete.





.. _MSP430Datasheet: https://www.ti.com/document-viewer/MSP430FR2155/datasheet/operating-modes-slasec45810#SLASEC45810