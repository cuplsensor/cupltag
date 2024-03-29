.. _StateChart:

State Chart
------------

This might look intimidating at first. Sub-sections cover the most common 
progressions through the state chart.

.. toctree::
   :maxdepth: 2
   
   unconfigured
   progmode
   firstrun
   samplingloop


.. uml::
   :caption: The state machine in :any:`main`.
   :width: 100%

   @startuml
        hide empty description
        [*] --> init_state

        init_state --> init_reqmemon: tr_ok

        init_reqmemon --> init_waitmemon: tr_ok

        init_waitmemon --> init_ntag: tr_ok
        init_waitmemon --> init_waitmemon: tr_wait

        init_ntag --> init_wakeupcheck: tr_ok
        init_ntag --> init_rtc_slow: tr_newconfig
        init_ntag --> init_progmode: tr_prog

        init_progmode --> init_progmode: tr_ok
        init_progmode --> init_progmode: tr_wait
        init_progmode --> err_msg: tr_fail

        init_wakeupcheck --> init_rtc_slow: tr_por
        init_wakeupcheck --> smpl_checkcounter: tr_samplingloop

        init_rtc_slow --> init_batvwait: tr_ok
        
        init_batvwait --> init_configcheck: tr_ok
        init_batvwait --> init_batvwait: tr_wait

        init_configcheck --> init_errorcheck: tr_ok
        init_configcheck --> [*]: tr_deepsleep

        init_errorcheck --> init_rtc_1min: tr_ok
        init_errorcheck --> [*]: tr_deepsleep

        init_rtc_1min --> smpl_checkcounter: tr_ok

        smpl_checkcounter --> smpl_hdcreq: tr_hdcreq
        smpl_checkcounter --> smpl_wait: tr_updatemin

        smpl_hdcreq --> smpl_hdcwait: tr_ok

        smpl_hdcwait --> smpl_hdcread: tr_ok
        smpl_hdcwait --> smpl_hdcwait: tr_wait

        smpl_hdcread --> smpl_wait: tr_ok
        smpl_hdcread --> [*]: tr_lowbat

        smpl_wait --> [*]: tr_deepsleep

        err_msg --> [*]: tr_deepsleep

        @enduml
