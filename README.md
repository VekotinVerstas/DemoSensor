# Demosensor

This sketch reads several environmental sensors
(e.g. humidity+thermometer or light meter), interrupts etc.
and sends the data to an MQTT broker to be saved into a database
or shown in some visualizations.

The software should run in any ESP8266 board, 
we've used Wemos D1 mini and mini Pro V1 and V2 lately.

# Getting started

## Create settings.h

First you must create settings.h with correct credentials:

`cp settings-example.h settings.h`

Then open `settings.h` and check that these defines have meaningful values:

`AP_PREFIX` and `BURK_ID` are used to generate hotspot name for WiFi configuration page.
`BURK_ID` is also sent with sensor data to the broker/server, so you have better control 
of Demosensor instance's indentity.

## Install required libraries

In order to complile Demosensor code you have to install so many libraries,
which are listed in the beginning of `Demosensor.ino`.
Carefully verify library name and version, if it is mentioned when installing.

If you know better way installing Arduino libraries than using 
Arduino IDE's awful library manager, let us know!

## Compile the code and flash ESP8266

Connect your ESP8266 to your PC's USB port, 
check that correct USB port is defined in Arduino IDE,
open Serial Monitor and then hit the Upload button.
If everyting works as expected, ESP8266 gets flashed and you should see some
text rolling in Serial Monitor window.

## Connecting to internet

Demosensor uses WLAN (WiFi) connection to send the data it gathers to an internet cloud server. 
When switched on, it tries to find pre-defined WLAN. 

If it can’t find it, Demosensor will create own WLAN hotspot called AP_PREFIX_BURK_ID, 
those strings are defined in `settings.h`. Use your PC, smart phone or tablet to connect this network.

![WiFi list](doc/img/wlan-list-mac.png?raw=true "WiFi list")

If connection is successful (sometimes you have to switch off and on your 
WLAN several times until connection works),
a popup window (a captive portal) or similar will open. 
First click Configure WiFi button:

![WiFiManager captive portal](doc/img/wifimanager-captive-portal.png?raw=true "WiFiManager captive portal")

And then in next window select desired WLAN by clicking it in the list and then set password in the lower form field. Double check that password is correct. You can write it down in clear text and then copy-paste it into the form.

![WiFiManager WiFi list in captive portal](doc/img/wifimanager-wlan-list-captive-portal.png?raw=true "WiFiManager WiFi list in captive portal")

# Sending data

Preferred protocol is MQTT, but it requires an MQTT broker service running on some server, 
so for initial testing there is HTTP POST option available.

## Data format 
Data format for both protocols is same. 

TODO: add examples
