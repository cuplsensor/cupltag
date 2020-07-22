.. _StateChart:

State Chart
~~~~~~~~~~~~~

.. uml::
   :caption: Caption with **bold** and *italic*
   :width: 50mm

    @startuml
    hide empty description
    [*] --> init_state

    init_state --> init_reqmemon: tr_ok

    init_reqmemon --> init_waitmemon: tr_ok

    init_waitmemon --> init_ntag: tr_ok
    init_waitmemon --> init_waitmemon: tr_wait

    init_ntag --> init_configcheck: tr_ok
    init_ntag --> init_progmode: tr_prog

    init_progmode --> init_progmode: tr_ok
    init_progmode --> init_progmode: tr_wait
    init_progmode --> err_reqmemon: tr_fail

    init_configcheck --> init_errorcheck: tr_ok
    init_configcheck --> [*]: tr_fail

    init_errorcheck --> init_rtc: tr_ok
    init_errorcheck --> [*]: tr_fail

    init_rtc --> smpl_checkcounter: tr_ok

    smpl_checkcounter --> smpl_hdcreq: tr_hdcreq
    smpl_checkcounter --> smpl_wait: tr_updatemin

    smpl_hdcreq --> smpl_hdcwait: tr_ok

    smpl_hdcwait --> smpl_hdcread: tr_ok
    smpl_hdcwait --> smpl_hdcwait: tr_wait

    smpl_hdcread --> smpl_wait: tr_ok

    smpl_wait --> rtc_reqmemon: tr_timeout
    smpl_wait --> smpl_wait: tr_wait

    rtc_reqmemon -->  rtc_waitmemon: tr_ok

    rtc_waitmemon --> smpl_checkcounter: tr_ok
    rtc_waitmemon --> rtc_waitmemon: tr_wait

    err_reqmemon --> err_waitmemon: tr_ok

    err_waitmemon --> err_msg: tr_ok
    err_waitmemon --> err_waitmemon: tr_wait

    err_msg --> [*]: tr_ok

    @enduml