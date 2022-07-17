.. _FirstRun:

First Run 
~~~~~~~~~~~~

The following state chart shows how the cuplTag operates the first time 
it is powered on with a good battery and a complete configuration. 

After some initialisation, a sample is collected from the HDC2021 and 
fed into cuplcodec.

.. uml::
   :caption: The state machine in :any:`main`.

   @startuml
        hide empty description
        [*] --> init_state

        init_state #83f795 --> init_reqmemon: tr_ok

        init_reqmemon #83f795 --> init_waitmemon: tr_ok

        init_waitmemon #83f795 --> init_ntag: tr_ok
        init_waitmemon --> init_waitmemon: tr_wait

        init_ntag #83f795 --> init_wakeupcheck: tr_ok
        init_ntag --> init_rtc_slow: tr_newconfig

        init_wakeupcheck --> init_rtc_slow: tr_por

        init_rtc_slow #83f795 --> init_batvwait: tr_ok
        
        init_batvwait #83f795--> init_configcheck: tr_ok
        init_batvwait --> init_batvwait: tr_wait

        init_configcheck #83f795 --> init_errorcheck: tr_ok
        init_configcheck --> [*]: tr_deepsleep

        init_errorcheck #83f795 --> init_rtc_1min: tr_ok
        init_errorcheck --> [*]: tr_deepsleep

        init_rtc_1min #83f795 --> smpl_checkcounter: tr_ok

        smpl_checkcounter #83f795 --> smpl_hdcreq: tr_hdcreq
        smpl_checkcounter --> smpl_wait: tr_updatemin

        smpl_hdcreq #83f795 --> smpl_hdcwait: tr_ok

        smpl_hdcwait #83f795 --> smpl_hdcread: tr_ok
        smpl_hdcwait --> smpl_hdcwait: tr_wait

        smpl_hdcread #83f795 --> smpl_wait: tr_ok
        smpl_hdcread --> [*]: tr_lowbat

        smpl_wait #83f795  --> [*]: tr_deepsleep

        @enduml
