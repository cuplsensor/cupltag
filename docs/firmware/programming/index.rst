.. _Programming:

Programming
~~~~~~~~~~~~~

.. image:: whatyouwillneed.jpg
  :width: 100%
  :alt: Items needed to program the MSP430

You will need:

* An `MSP-FET <https://www.mouser.co.uk/ProductDetail/Texas-Instruments/MSP-FET?qs=Mrfp3zus3mNXSLrVqFkg8A==>` and a USB cable.
* A PC running Code Composer Studio.
* 4 coloured jumper wires.
* A 2x4 way 2.54mm pitch pin header.
* A 1x2 way 2.54mm pitch pin header.
* A 2 way jumper.
* Solder.
* A cuplTag PCBA (HT07), unscrewed from the enclosure, with no battery inserted.

.. image:: fetschematic.jpg
  :width: 100%
  :alt: MSP-FET Spy-Bi-Wire Schematic

We will program / debug the MSP430 on HT07 using Spy-Bi-Wire. Connect it to the MSP-FET.

+---------+--------+--------------+-------------+--------------+---------------+
| Name    | Colour | MSP-FET name | MSP-FET pin | HT07 J30 pin | HT07 J30 name |
+---------+--------+--------------+-------------+--------------+---------------+
| +3V3    | Red    | VCC_TOOL     | 2           | 7            | VDD           |
+---------+--------+--------------+-------------+--------------+---------------+
| GND     | Black  | GND          | 9           | 3            | GND           |
+---------+--------+--------------+-------------+--------------+---------------+
| SBWTDIO | White  | TDO/TDI      | 1           | 6            | nRST          |
+---------+--------+--------------+-------------+--------------+---------------+
| SBWTCK  | Purple | TCK          | 7           | 4            | TST           |
+---------+--------+--------------+-------------+--------------+---------------+

.. image:: fetconnections.jpg
  :width: 100%
  :alt: Jumper wire connections on the MSP-FET

Connect the MSP-FET to a PC with a USB cable. Open the Code Composer Studio cuplTag project `created earlier <GettingStarted>`.



