Error Conditions
-----------------

In some cases normal operation is not be possible. The following procedure is followed:

#. An error message is written to the tag. This is either:

    * An NDEF text record with a description of the error.
    * The cuplCodec URL record without the circular buffer.

#. The tag ceases operation and enters LPM4 (deep sleep). This conserves battery life until the user
   discovers the error.

Configuration Check Failed
~~~~~~~~~~~~~~~~~~~~~~~~~~~

After cuplTag has been erased and programmed anew, the variable :cpp:member:`writtenfields` is zero. Each
time a configuration string is received and written to non-volatile memory, its corresponding bit is set.

For example, bit 1 is set when the secret key is received.

When all of the required bits have been set in writtenfields, the variable allwritten in non-volatile memory is
cleared.

At startup :cpp:func:`nvparams_allwritten` returns 1 if allwritten is cleared. This means that all configuration
strings have been set in order for the cuplCodec Encoder to run. If this is not the case then the program cannot
continue. 

No URL can be written to the tag if the baseURL field is blank. It is not sensible to have a default URL
because it cannot be guaranteed that the URL will point to a web server forever. Therefore the only option is
to write a short NDEF text record: ``Config check failed. See cuplTag documentation.``
The user will then be able to refer to this documentation to find out how to write all configuration strings.