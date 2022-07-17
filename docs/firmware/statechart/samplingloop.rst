.. _SamplingLoop:

Sampling Loop
~~~~~~~~~~~~~~~

The cuplTag wakes up each minute. Several initialisation states are 
skipped. The minute counter is incremented. If this equals the configured 
sampling interval, then a sample is collected from the HDC2021.

cuplcodec updates the minute counter in the cupl URL (stored on the NFC EEPROM). 
If a new sample is available, this is encoded and added to the circular buffer.

.. uml::
   :caption: The state machine runs each minute in the sampling loop.
   :width: 100%

   @startuml
        hide empty description
        [*] --> init_state

        init_state #83f795 --> init_reqmemon: tr_ok

        init_reqmemon #83f795 --> init_waitmemon: tr_ok

        init_waitmemon #83f795 --> init_ntag: tr_ok
        init_waitmemon --> init_waitmemon: tr_wait

        init_ntag #83f795 --> init_wakeupcheck: tr_ok

        init_wakeupcheck #83f795 --> smpl_checkcounter: tr_samplingloop

        smpl_checkcounter #83f795  --> smpl_hdcreq: tr_hdcreq
        smpl_checkcounter --> smpl_wait: tr_updatemin

        smpl_hdcreq #83f795 --> smpl_hdcwait: tr_ok

        smpl_hdcwait #83f795  --> smpl_hdcread: tr_ok
        smpl_hdcwait --> smpl_hdcwait: tr_wait

        smpl_hdcread #83f795 --> smpl_wait: tr_ok
        smpl_hdcread --> [*]: tr_lowbat

        smpl_wait #83f795  --> [*]: tr_deepsleep

        err_msg  --> [*]: tr_deepsleep

        @enduml
