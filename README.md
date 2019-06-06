
# DripIrrigationCtrl

Smart Control for Drip Irrigation

This project is aimed to control vegetable garden drip irrigation by setting start dripping time, duration, frequency, rain delay, etc. It also reports water flow measurements.

## Features

* MQTT enabled. Setup and control drip irrigation settings by using MQTT commands.

* LCD Display. Display time and system status on a 16x2 Liquid Crystal Display

* Push Button. Restart system, restart Wi-Fi settings, manual start/stop dripping, and set rain delay using a single push button.

* LED indication. Indicates initialization, normal status, and dripping by flashing LED.

* Peristent Setting Storage. Dripping settings are persistently stored in EEPROM

* OTA. Over the air update is enabled by default.

## Operation

* First time operation. Power the system and connect to the AP (esp8266/esp8266) and navigate to the ip address showed in the display. Setup your WiFi SSID and password so the system can connect to your network.

* Upon restart the system initializes by connecting to the Wi-Fi, setup a connection to the MQTT broker and schedule next dripping period (or start dripping) based on default dripping settings.

* Push Button Operation:

  * Short Push (less than 3 seconds) will start/stop dripping
  * Medium push (less than 8 seconds) will set a 24 hours rain delay.
  * Long Push (more than 8 seconds) will restart the system
  * Power up while pushing button will reset Wi-Fi settings and start the system in AP mode.

* MQTT Commands:

  By default the system subscribes to MQTT topic /home-assistant/drip/request. The payload includes the command (first byte), and the data (subsequent bytes)
  * Dripping Settings Payload: cHH:MM:SSMMHH where HH:MM:SS (24 hours format) is start time, MM duration (in minutes), and HH period (in hours)
  * Rain Delay Payload: rHH where HH is the rain delay in hours
  * Start Dripping Payload: sMM where MM is the dripping time in minutes
  * Stop Dripping Paylod:  t
  * Reset Payload: x
  
  The system will also report using the following MQTT command:

  * /home-assistant/drip/flow the payload will have the metered water flow after each dripping cycle. Payload: xxx where xxx is the total liters.
  * /home-assistant/drip/started drip has started. No payload.
  * /home-assistant/drip/stopped drip has stopped. No payload.

## Schemmatic
