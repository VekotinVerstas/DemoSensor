This sketch reads several environmental sensors
(e.g. humidity+thermometer or light meter), interrupts etc.
and sends the data to an MQTT broker to be saved into a database
or shown in some visualizations.

Note: you must create settings.h with correct credentials:

`cp settings-example.h settings.h  # then edit settings.h`
