# cuplTag
The cuplTag firmware runs on an [MSP430FR2155](https://www.ti.com/product/MSP430FR2155). It collects samples from an [HDC2021](https://www.ti.com/product/HDC2021) sensor 
and passes it to the encoder part of [cuplcodec](https://github.com/cuplsensor/cuplcodec) at a used-defined time interval. The output is a URL. This is written to an NFC EEPROM 
([NT3H2111](https://www.nxp.com/docs/en/data-sheet/NT3H2111_2211.pdf)), which is connected to a Type 6 antenna. 
The URL origin is configurable. 

Configuration parameters are written over a serial port at 9600 baud. Alternatively, these are transmitted in an NFC text record.

The firmware spends much of its time in LPM3 in order to achieve an average current consumption ~1.5uA, which gives years of operation from a coin cell battery.

## Documentation 

[![Documentation Status](https://readthedocs.org/projects/wsbackend/badge/?version=latest)](https://cupl.readthedocs.io/projects/backend/en/latest/?badge=latest) 

Hosted on [ReadTheDocs](https://cupl.readthedocs.io/projects/backend/en/latest/). This includes information on how to install the software from scratch without Docker.
    
## Licence

### Firmware

[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

### Hardware

[CERN Open Hardware Licence Version 2 - Strongly Reciprocal](https://ohwr.org/cern_ohl_s_v2.txt)

### Documentation

[![License: CC BY-SA 4.0](https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-sa/4.0/)
