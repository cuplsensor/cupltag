State Chart
~~~~~~~~~~~~~

.. uml::
   :caption: Caption with **bold** and *italic*
   :width: 50mm

    @startuml
    hide empty description
    [*] --> init_state

    init_state --> init_reqsyson: tr_ok

    init_reqsyson --> init_waitsyson: tr_ok

    init_waitsyson --> init_ntag_and_fd: tr_ok
    init_waitsyson --> init_waitsyson: tr_wait

    init_ntag_and_fd --> init_nfccheck: tr_ok

    init_nfccheck --> init_serialcheck: tr_ok

    init_serialcheck --> init_errorcheck: tr_ok
    init_serialcheck --> init_serialcheck: tr_fail

    init_errorcheck --> init_rtc: tr_ok
    init_errorcheck --> [*]: tr_fail

    init_rtc --> smpl_checkcounter: tr_ok

    smpl_checkcounter --> smpl_hdcreq: tr_hdcreq
    smpl_checkcounter --> smpl_wait: tr_updatemin

    smpl_hdcreq --> smpl_hdcwait: tr_ok

    smpl_hdcwait --> smpl_hdcread: tr_ok
    smpl_hdcwait --> smpl_hdcwait: tr_wait

    smpl_hdcread --> smpl_wait: tr_ok
    smpl_hdcread --> [*]: tr_fail


    smpl_wait --> rtc_reqsyson: tr_ok
    smpl_wait --> smpl_wait: tr_wait

    rtc_reqsyson -->  rtc_waitsyson: tr_ok

    rtc_waitsyson --> smpl_checkcounter: tr_ok
    rtc_waitsyson --> rtc_waitsyson: tr_wait

    @enduml