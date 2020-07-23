Error Conditions
-----------------

Before entering :ref:`normal operation <NormalMode>` some checks are made. If any of these fail:

#. An error message is written to the dynamic tag. This is either:

    * An NDEF text record with a description of the error.
    * The cuplCodec URL record without the circular buffer.

#. cuplTag shuts down by entering LPM4 (deep sleep).

Battery life is conserved until the user attempts to read the tag and discovers the error.

Configuration Check Failed
~~~~~~~~~~~~~~~~~~~~~~~~~~~

After cuplTag has been erased and programmed anew, the variable in non-volatile memory :cpp:member:`allwritten`
is 1. Each time a valid configuration string is received, its corresponding bit is set in the
RAM-based variable :cpp:member:`writtenfields`.

For example:

* Bit 0 is set in response to serial string.
* Bit 1 is set in response to the secret key.

When all required bits have been set in ``writtenfields``, its non-volatile counterpart :cpp:member:`allwritten`
is cleared.

At startup :cpp:func:`nvparams_allwritten` returns 1 if :cpp:member:`allwritten` is cleared.
This means that all configuration strings have been set in order for the cuplCodec Encoder to run.

The error cannot be communicated by writing a URL to the dynamic tag. The base URL field has
not been written and there is no guarantee a default URL will point to a web server.

A short NDEF text record is written instead: ``Config check failed. See cuplTag documentation.``

Voltage Check Failed
~~~~~~~~~~~~~~~~~~~~~

cuplTag measures the voltage and will not continue if it is below a configurable threshold.


Repeated Resets caused by an Error
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Flash memory on the dynamic tag must be protected from repeated writes. This may occur if a fault occurs repeatedly
that causes a reset. For example:

#. A brownout reset occurs whilst the dynamic tag is been written.
#. The tag resets and start to write to the dynamic tag again. The reset reoccurs.

If unchecked this cycle can go around many times each second. This will cause the dynamic tag to have worn out
before the fault can be addressed. The cuplTag employs a "last ditch" protection feature to avoid this.

Invalid State Transition
~~~~~~~~~~~~~~~~~~~~~~~~~~

This error may be encountered by a programmer but should never be seen by an end-user!

A function should never request a state transition that is not defined in the state table.

If this does happen, a catch-all state is entered cpp:func:``err_reqmemon``.
The dynamic tag is powered up and an NDEF text record is written:
``Invalid state transition``.

cuplTag subsequently enters its end state and powers down to LPM4.