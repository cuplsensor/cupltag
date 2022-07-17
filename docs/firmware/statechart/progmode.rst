.. _ProgMode:

Programming Mode
~~~~~~~~~~~~~~~~~~

.. uml::
   :caption: The state machine branches to :any:`init_progmode` when the nPRG pin is LOW.

   @startuml
        hide empty description
        [*] --> init_state

        init_state #83f795 --> init_reqmemon: tr_ok

        init_reqmemon #83f795 --> init_waitmemon: tr_ok

        init_waitmemon #83f795 --> init_ntag: tr_ok
        init_waitmemon --> init_waitmemon: tr_wait

        init_ntag #83f795 --> init_progmode: tr_prog

        init_progmode #83f795 --> init_progmode: tr_ok
        init_progmode --> init_progmode: tr_wait

        @enduml
