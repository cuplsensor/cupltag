.. _Unconfigured:

Not Configured
~~~~~~~~~~~~~~~

.. uml::
   :caption: Startup does not continue when the configuration is incomplete.

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

        init_rtc_slow #83f795  --> init_batvwait: tr_ok
        
        init_batvwait #83f795 --> init_configcheck: tr_ok
        init_batvwait --> init_batvwait: tr_wait

        init_configcheck #83f795  --> [*]: tr_deepsleep

        @enduml
