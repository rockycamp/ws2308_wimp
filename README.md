#WS2308 Internet Bridge#

This is a project based on the Sparkfun WIMP personal weather station:
https://github.com/sparkfun/Wimp_Weather_Station

The aim was to adapt this project to bridge an existing WS2308 station to Weather Underground.

The weather station is queried by an Arduino Uno using a level shifter and the captured data is sent to an Electric Imp via serial where it is relayed to an agent and then to Weather Underground servers.
