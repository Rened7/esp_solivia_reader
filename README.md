# ESP8266 Micropython Solivia Reader

Thanks to the https://github.com/lvzon/soliviamonitor/ project which has provided the crc checking code.
Thanks to the https://github.com/bbinet/delta-rpi/ project which has provided the basis for the core function of commmunicating and formatting data.

ESP Solivia Reader is a micropython script that runs on an ESP8266 to gather solar data from a Delta Solivia inverter and store the data in InfluxDB.
The code can be configure to work on all Delta Solar inverters that are listed in the Public RS485 Protocol 1V2.pdf.
However this has not been tested as I only have access to my own which is a Solivia 5.0 AP G3.

## Setup

A MQTT broker will be required to be setup on your network as well as InfluxDB.
I am using telegraf to get the data from the MQTT broker to send to InfluxDB and a configure can be found in the Telegraf folder.
With the scripts for the ESP main.py, delta_esp.py, crc16.mpy and umqttsimple.mpy need to be sent to the ESP and will autorun on reboot.

## ESP8266

For the ESP I used a Wemos D1 mini and connected a MAX 485 for RS485 communication to Inverter.
