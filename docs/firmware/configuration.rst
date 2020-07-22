Configuration
----------------

Mechanisms
~~~~~~~~~~~~

Serial
**********

Configuration string can be transmitted as commands in :ref:`Programming Mode <ProgMode>`.

NFC
*******

A message containing one NDEF text record can be written to the tag. A number of 3rd party apps NFC writer
apps are capable of this. In addition, this can be done using WebNFC using wsfrontend.

The text record is comprised of one or more configuration strings described below. The cuplTag firmware
checks for the presence of this record each minute (see :ref:`Normal Operation <NormalMode>`). This consumes
negligible power because the VMEM domain is powered up for writing the NTAG anyway. It is also simpler than
using the Field Detect interrupt from the NTAG.

If the text record is found, a soft reset is triggered and the text record is parsed at startup;
any new parameters written to non-volatile memory before use by cuplCodec.

Configuration Strings
~~~~~~~~~~~~~~~~~~~~~~~

Base URL
*********

Serial
********

Sample Interval in Minutes
****************************

