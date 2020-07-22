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

+------------------+---------------------+
| Command ID       | b                   |
+------------------+---------------------+
| Parameter Length | up to 64            |
+------------------+---------------------+
| Parameter value  | Any string          |
+------------------+---------------------+

Example: ``<b:localhost:5000>``

Serial
********

+------------------+---------------------+
| Command ID       | w                   |
+------------------+---------------------+
| Parameter Length | 8                   |
+------------------+---------------------+
| Parameter value  | Any URL-safe Base64 |
+------------------+---------------------+

Example: ``<w:KEG2lARW>``

Sample Interval in Minutes
****************************


HMAC Secret Key
*****************

+------------------+---------------------+
| Command ID       | s                   |
+------------------+---------------------+
| Parameter Length | 16                  |
+------------------+---------------------+
| Parameter value  | Any URL-safe Base64 |
+------------------+---------------------+

Example: ``<s:4EOBdBWTsjeFZTm3>``

Use HTTPS
***********

+------------------+---------------------------------------+
| Command ID       | h                                     |
+------------------+---------------------------------------+
| Parameter Length | 1                                     |
+------------------+---------------------------------------+
| Parameter value  | 0 (HTTPS disabled), 1 (HTTPS enabled) |
+------------------+---------------------------------------+

Example: ``<h:1>``


Use HMAC
************

+------------------+---------------------------------------+
| Command ID       | i                                     |
+------------------+---------------------------------------+
| Parameter Length | 1                                     |
+------------------+---------------------------------------+
| Parameter value  | 0 (HMAC disabled), 1 (HMAC enabled)   |
+------------------+---------------------------------------+

Example ``<i:1>``
